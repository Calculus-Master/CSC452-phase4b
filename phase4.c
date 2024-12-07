#include <phase1.h>
#include <phase2.h>
#include <phase3.h>
#include <phase4.h>
#include <phase3_kernelInterfaces.h>
#include <usloss.h>
#include <stdio.h>
#include <string.h>

// Data Structures

typedef struct ShadowProcess {
    int pid;
    int target_time;

    struct ShadowProcess* next;
} ShadowProcess;

ShadowProcess process_table[MAXPROC];
ShadowProcess* sleep_queue;

typedef struct TerminalData {
    // Reading
    int read_mbox_id; // Holds the 10 terminal buffers
    int read_semaphore_id; // Semaphore lock for terminal read operations
    char working_buffer[MAXLINE + 1]; // Current working buffer
    int working_buffer_index; // Index of first free space in working buffer
    int is_reading; // Flag to indicate if a read is in progress

    // Writing
    int write_semaphore_id; // Semaphore lock for terminal write operations
    int daemon_mutex; // Mutex between daemon writing and the syscall TermWrite
    int write_mbox_id; // Holds characters to be written to terminal
    char write_buffer[MAXLINE + 1]; // Buffer for characters waiting to be written to terminal
    int write_buffer_index; // Index of the next character waiting to be written
    int is_writing; // Flag to indicate if a write is in progress
} TerminalData;

typedef struct DiskOpRequest {
    int pid;
    int is_write; // 0: Read, 1: Write

    int start_track;
    int start_block;
    int num_blocks;

    char* buffer;

    int mbox_id; // Mailbox to send the result of the operation
    struct DiskOpRequest* next;
} DiskOpRequest;

typedef struct DiskData {
    int access_mutex; // Semaphore controlling mutex access to disk operations
    int idle_mbox; // Mailbox for the daemon to sleep on when no operations are pending
    int track_count; // Number of tracks on the disk
    USLOSS_DeviceRequest current_request;

    int queue_mutex;
    DiskOpRequest* user_request_queue;
    int current_track;
} DiskData;

TerminalData terminal_data[USLOSS_TERM_UNITS];
DiskData disk_data[USLOSS_DISK_UNITS];
DiskOpRequest disk_user_requests[MAXPROC];

// Helper Functions

// Clears the working read buffer for the specified terminal
void clear_working_buffer(TerminalData* data)
{
    memset(data->working_buffer, 0, MAXLINE + 1);
    data->working_buffer_index = 0;
}

// Enables read interrupts for the specified terminal
void enable_terminal_reads(int unit)
{
    int control = 0; // Make sure 'send char' is off (not sending any data)
    control |= 0x2; // Turn on read interrupt

    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void*)(long)control);
}

// Enables write interrupts for the specified terminal (DOES NOT SEND A CHARACTER)
void enable_terminal_writes(int unit)
{
    int control = 0; // Make sure 'send char' is off (not sending any data)
    control |= 0x2; // Turn on read interrupt
    control |= 0x4; // Turn on write interrupt

    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void*)(long)control);
}

// Writes a single character to the specified terminal, turning on the valid bits (as stated in Terminal supplement)
void write_terminal_character(int unit, char c)
{
    int control = 0x1; // Turn on 'send char'
    control |= 0x2; // Turn on read interrupt
    control |= 0x4; // Turn on write interrupt
    control |= (c << 8); // Send char

    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void*)(long)control);
}

int perform_disk_seek(int unit, int target_track)
{
    USLOSS_DeviceRequest req;
    memset(&req, 0, sizeof(USLOSS_DeviceRequest));

    req.opr = USLOSS_DISK_SEEK;
    req.reg1 = target_track;
    req.reg2 = 0;

    USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &req);

    int status;
    waitDevice(USLOSS_DISK_DEV, unit, &status);

    return status;
}

int perform_disk_rw(int unit, DiskOpRequest* request)
{
    int block = request->start_block;
    int track = request->start_track;
    int blocks_completed = 0;
    char* current_buffer = request->buffer;

    while(blocks_completed < request->num_blocks)
    {
        // Read or write current block
        USLOSS_DeviceRequest dev_req;
        memset(&dev_req, 0, sizeof(USLOSS_DeviceRequest));

        dev_req.opr = request->is_write ? USLOSS_DISK_WRITE : USLOSS_DISK_READ;
        dev_req.reg1 = block;
        dev_req.reg2 = current_buffer;

        USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &dev_req);

        int status;
        waitDevice(USLOSS_DISK_DEV, unit, &status);

        if(status != USLOSS_DEV_OK)
            return status;

        block++;
        blocks_completed++;
        current_buffer += USLOSS_DISK_SECTOR_SIZE;

        if(block >= USLOSS_DISK_TRACK_SIZE) // Move to the next track
        {
            block = 0;
            track++;

            status = perform_disk_seek(unit, track);
            if(status != USLOSS_DEV_OK)
                return status;
        }
    }

    return 0;
}

// Does CSCAN processing for the disk operations queue
void cscan_disk(int unit, DiskData* disk)
{
    while(disk->user_request_queue != NULL)
    {
        // Select request with next highest track
        DiskOpRequest* prev = NULL;
        DiskOpRequest* current = disk->user_request_queue;

        DiskOpRequest* target_prev = NULL;
        DiskOpRequest* target;
        int diff = 100000;

        while(current != NULL)
        {
            int track_diff = current->start_track - disk->current_track;
            if(track_diff >= 0 && track_diff < diff) // New next best track
            {
                target_prev = prev;
                target = current;
                diff = track_diff;
            }

            prev = current;
            current = current->next;
        }

        // If target is still null -> seek to track 0
        if(target == NULL)
        {
            perform_disk_seek(unit, 0);
            continue; // Restart the loop
        }
        else // Perform the selected disk operation
        {
            // SEEK
            int status = perform_disk_seek(unit, target->start_track);

            // READ/WRITE (only if the seek succeeded)
            if(status == USLOSS_DEV_OK)
                status = perform_disk_rw(unit, target);

            // Remove operation from request queue
            if(target_prev == NULL)
                disk->user_request_queue = target->next;
            else
                target_prev->next = target->next;

            // Wake up the process
            MboxSend(target->mbox_id, &status, sizeof(int));
        }
    }
}

// Device driver for sleeping processes, handles clock interrupts continuously
int sleep_daemon_process()
{
    while(1)
    {
        // Wait for clock interrupt to fire
        int status;
        waitDevice(USLOSS_CLOCK_DEV, 0, &status);

        // Loop through and wakeup any sleeping processes that have reached their target time
        int has_proc = 1;
        while(has_proc)
        {
            ShadowProcess* sleep_proc = sleep_queue;

            // Stop looping if there are no more sleeping processes
            if(sleep_proc == NULL)
                has_proc = 0;
            else
            {
                int current_time = currentTime();

                if(sleep_proc->target_time <= current_time)
                {
                    // Wakeup the process
                    unblockProc(sleep_proc->pid);

                    // Remove from queue and clear process data
                    sleep_queue = sleep_proc->next;
                    memset(sleep_proc, 0, sizeof(ShadowProcess));
                }
                else // Next process is not ready to wake up, so stop looping
                    has_proc = 0;
            }
        }
    }

    return 0; // Should never reach this point
}

// Device driver for a terminal, handles Read and Write interrupts continuously
int terminal_daemon_process(void* arg)
{
    int unit = (int)(long)arg;
    TerminalData* term = &terminal_data[unit];

    enable_terminal_reads(unit);

    while(1)
    {
        // Wait for the current terminal interrupt to fire
        int status;
        waitDevice(USLOSS_TERM_DEV, unit, &status);

        // Handle reads
        int read_status = USLOSS_TERM_STAT_RECV(status);

        if(read_status == USLOSS_DEV_ERROR) // Should not happen, but this is a failsafe
        {
            USLOSS_Console("Terminal %d (Read): Error from waitDevice status.\n", unit);
            USLOSS_Halt(USLOSS_DEV_ERROR);
        }
        else if(read_status == USLOSS_DEV_READY) {} // Do nothing for reads
        else if(read_status == USLOSS_DEV_BUSY)
        {
            kernSemP(term->read_semaphore_id);

            char c = USLOSS_TERM_STAT_CHAR(status);

            term->working_buffer[term->working_buffer_index++] = c;

            // End of line reached
            if(c == 0 || c == '\n' || term->working_buffer_index == MAXLINE)
            {
                term->working_buffer[term->working_buffer_index] = '\0';

                // Send the line to the mailbox (if mailbox is full, it gets deleted)
                MboxCondSend(term->read_mbox_id, term->working_buffer, strlen(term->working_buffer));

                clear_working_buffer(term);
                term->is_reading = 0;
            }

            kernSemV(term->read_semaphore_id);
        }

        // Handle writes
        int write_status = USLOSS_TERM_STAT_XMIT(status);

        if(write_status == USLOSS_DEV_ERROR) // Should not happen, but this is a failsafe
        {
            USLOSS_Console("Terminal %d (Write): Error from waitDevice status.\n", unit);
            USLOSS_Halt(USLOSS_DEV_ERROR);
        }
        else if(write_status == USLOSS_DEV_BUSY) {} // Do nothing for writes
        else if(write_status == USLOSS_DEV_READY)
        {
            // Active write operation
            if(term->is_writing)
            {
                char c = term->write_buffer[term->write_buffer_index++];
                write_terminal_character(unit, c);

                // Reached end of input
                if(term->write_buffer[term->write_buffer_index] == '\0')
                {
                    term->is_writing = 0;
                    memset(term->write_buffer, 0, MAXLINE + 1);
                    term->write_buffer_index = 0;

                    kernSemV(term->daemon_mutex);
                }
            }

        }
    }

    return 0; // Should never reach this point
}

// Device driver for the disk
int disk_daemon_process(void* arg)
{
    int unit = (int)(long)arg;
    DiskData* disk = &disk_data[unit];

    // Query track count initially
    // Until this operation completes, the access mutex is not released so no other disk operations can
    // occur until the track count is known

    int track_count;
    disk->current_request.opr = USLOSS_DISK_TRACKS;
    disk->current_request.reg1 = &track_count;
    disk->current_request.reg2 = 0;

    USLOSS_DeviceOutput(USLOSS_DISK_DEV, unit, &disk->current_request);

    // Wait for the query to end
    int status;
    waitDevice(USLOSS_DISK_DEV, unit, &status);

    // Update the received values and clear the request
    disk->track_count = track_count;
    memset(&disk->current_request, 0, sizeof(USLOSS_DeviceRequest));

    // Release the access mutex so syscalls can start to complete
    kernSemV(disk->access_mutex);

    // Standard infinite loop to handle disk ops
    while(1)
    {
        kernSemP(disk->queue_mutex);

        if(disk->user_request_queue != NULL) // Pending disk ops to handle
        {
            // CSCAN processing until the queue is empty
            cscan_disk(unit, disk);

            kernSemV(disk->queue_mutex);
        }
        else // No pending disk ops, so block until one arrives
        {
            kernSemV(disk->queue_mutex);

            MboxRecv(disk->idle_mbox, NULL, 0);
        }
    }

    return 0;
}

// Syscall Handlers

// Handles the sleep syscall
void sleep_handler(USLOSS_Sysargs* args)
{
    int seconds = (int)(long)args->arg1;

    if(seconds < 0) // Invalid number of seconds
        args->arg4 = (void*)-1;
    else
    {
        // Setup shadow process struct
        ShadowProcess* sleep_proc = &process_table[getpid() % MAXPROC];
        sleep_proc->pid = getpid();

        // Calculate target time
        int current_time = currentTime(); // µs
        sleep_proc->target_time = current_time + seconds * 1000 * 1000;

        // Add to sleep queue
        if(sleep_queue == NULL)
            sleep_queue = sleep_proc;
        else
        {
            ShadowProcess* current = sleep_queue;
            while(current->next != NULL)
                current = current->next;

            current->next = sleep_proc;
        }

        // Block the process
        blockMe();

        args->arg4 = (void*)0;
    }
}

// Handles the TermRead syscall
void term_read_handler(USLOSS_Sysargs* args)
{
    int unit = (int)(long)args->arg3; // Terminal number
    char* buffer = (char *)args->arg1; // User buffer
    int bufSize = (int)(long)args->arg2; // Buffer size

    if (unit < 0 || unit >= USLOSS_TERM_UNITS || buffer == NULL || bufSize <= 0)
    {
        args->arg2 = (void *)0; // No characters read
        args->arg4 = (void *)-1; // Invalid arguments
        return;
    }

    TerminalData* term = &terminal_data[unit];
    term->is_reading = 1;

    // Create a temporary buffer to receive the terminal message
    char temp[MAXLINE + 1];
    memset(temp, 0, MAXLINE + 1);

    // Reads the first available line, or blocks until the daemon process sends a line
    int length = MboxRecv(term->read_mbox_id, temp, MAXLINE);

    // Copy the message to the user buffer, truncating if needed
    strncpy(buffer, temp, bufSize);

    if(bufSize < length)
        length = bufSize;

    args->arg2 = (void *)(long) length; // Characters read
    args->arg4 = (void *)0; // Success
}

// Handles the TermWrite syscall
void term_write_handler(USLOSS_Sysargs* args)
{
    int unit = (int)(long)args->arg3; // Terminal number
    char* buffer = (char *)args->arg1; // User buffer
    int bufSize = (int)(long)args->arg2; // Buffer size

    if (unit < 0 || unit >= USLOSS_TERM_UNITS || buffer == NULL || bufSize <= 0)
    {
        args->arg4 = (void *)-1; // Invalid arguments
        return;
    }

    TerminalData* term = &terminal_data[unit];

    // Grab semaphore lock
    kernSemP(term->write_semaphore_id);

    // Copy the buffer to the terminal
    memset(term->write_buffer, 0, MAXLINE + 1);
    strncpy(term->write_buffer, buffer, bufSize + 1);
    term->write_buffer_index = 0;

    // Enable writes in the Control Register and enable the flag for the daemon
    enable_terminal_writes(unit);
    term->is_writing = 1;

    // Block until the daemon finishes writing
    kernSemP(term->daemon_mutex);

    args->arg2 = (void *)(long)bufSize; // Characters written
    args->arg4 = (void *)0; // Success

    // Release semaphore lock
    kernSemV(term->write_semaphore_id);
}

// Handles the DiskSize syscall
void disk_size_handler(USLOSS_Sysargs* args)
{
    int unit = (int)(long)args->arg1;

    if(unit < 0 || unit >= USLOSS_DISK_UNITS) // Invalid disk unit
    {
        args->arg4 = (void*)-1;
        return;
    }

    DiskData* disk = &disk_data[unit];

    // Acquire access mutex
    // If this runs before the daemon finishes the tracks query, it'll just block here
    kernSemP(disk->access_mutex);

    int tracks = disk->track_count;

    args->arg1 = (void*)(long)USLOSS_DISK_SECTOR_SIZE; // Sector size
    args->arg2 = (void*)(long)USLOSS_DISK_TRACK_SIZE; // Number of sectors in a track
    args->arg3 = (void*)(long)tracks; // Number of tracks in the disk
    args->arg4 = (void*)(long)0; // Success

    // Release access mutex
    kernSemV(disk->access_mutex);
}

void disk_rw_common(USLOSS_Sysargs* args, int is_write)
{
    char* buffer = (char*)args->arg1;
    int num_blocks = (int)(long)args->arg2;
    int start_track = (int)(long)args->arg3;
    int start_block = (int)(long)args->arg4;
    int unit = (int)(long)args->arg5;

    // Invalid arguments (unit)
    if(unit < 0 || unit >= USLOSS_DISK_UNITS)
    {
        args->arg4 = (void*)-1;
        return;
    }

    DiskData* disk = &disk_data[unit];

    // Invalid arguments (everything else)
    if(buffer == NULL || num_blocks <= 0 ||
        start_track < 0 ||
        start_block < 0 || start_block >= USLOSS_DISK_TRACK_SIZE)
    {
        args->arg4 = (void*)-1;
        return;
    }

    // Create a disk operation request
    DiskOpRequest* request = &disk_user_requests[getpid() % MAXPROC];
    request->pid = getpid();
    request->is_write = is_write;
    request->start_track = start_track;
    request->start_block = start_block;
    request->num_blocks = num_blocks;
    request->buffer = buffer;
    request->mbox_id = MboxCreate(1, sizeof(int));

    // Add to the user request queue
    kernSemP(disk->queue_mutex);

    if(disk->user_request_queue == NULL)
        disk->user_request_queue = request;
    else
    {
        DiskOpRequest* current = disk->user_request_queue;
        while(current->next != NULL)
            current = current->next;

        current->next = request;
    }

    kernSemV(disk->queue_mutex);

    // Wake up daemon if its idle
    MboxCondSend(disk->idle_mbox, NULL, 0);

    // Wait for the daemon to finish handling the request
    int status;
    MboxRecv(request->mbox_id, &status, sizeof(int));

    // Delete the mailbox since it's no longer needed
    MboxRelease(request->mbox_id);

    // Clear the request and assign proper output, delete the mailbox
    memset(request, 0, sizeof(DiskOpRequest));
    args->arg1 = (void*)(long)status; // Either 0 or device status register
    args->arg4 = (void*)0;
}

// Handles the DiskRead syscall
void disk_read_handler(USLOSS_Sysargs* args)
{
    disk_rw_common(args, 0);
}

// Handles the DiskWrite syscall
void disk_write_handler(USLOSS_Sysargs* args)
{
    disk_rw_common(args, 1);
}

// Phase 4 Functions

// Sets up data structures and objects for stuff in Phase 4
void phase4_init()
{
    // Setup sleeping process structs
    sleep_queue = NULL;
    memset(process_table, 0, sizeof(ShadowProcess) * MAXPROC);

    // Assign syscall handlers
    systemCallVec[SYS_SLEEP] = sleep_handler;
    systemCallVec[SYS_TERMREAD] = term_read_handler;
    systemCallVec[SYS_TERMWRITE] = term_write_handler;
    systemCallVec[SYS_DISKSIZE] = disk_size_handler;
    systemCallVec[SYS_DISKREAD] = disk_read_handler;
    systemCallVec[SYS_DISKWRITE] = disk_write_handler;

    // Setup terminal r/w data structures
    memset(terminal_data, 0, sizeof(TerminalData) * USLOSS_TERM_UNITS);
    for(int i = 0; i < 4; i++)
    {
        TerminalData* term = &terminal_data[i];

        // Create Terminal Read Mailbox
        int mbox_id = MboxCreate(10, MAXLINE + 1);
        term->read_mbox_id = mbox_id;

        // Create Terminal Read Semaphores
        kernSemCreate(1, &term->read_semaphore_id);

        // Empty Working Buffers
        clear_working_buffer(term);

        // Create Terminal Write Semaphores
        kernSemCreate(1, &term->write_semaphore_id);
        kernSemCreate(0, &term->daemon_mutex);

        // Create Terminal Write Mailbox
        mbox_id = MboxCreate(1, 1);
        term->write_mbox_id = mbox_id;

        // Disable the terminal itself to start
        USLOSS_DeviceOutput(USLOSS_TERM_DEV, i, (void*)(long)0);
    }

    // Setup disk data structures
    memset(disk_data, 0, sizeof(DiskData) * USLOSS_DISK_UNITS);
    for(int i = 0; i < 2; i++)
    {
        DiskData* disk = &disk_data[i];

        disk->idle_mbox = MboxCreate(0, 0);

        // Create disk access mutex semaphore
        kernSemCreate(0, &disk->access_mutex);

        // Create disk queue mutex semaphore
        kernSemCreate(1, &disk->queue_mutex);
    }
    memset(disk_user_requests, 0, sizeof(DiskOpRequest) * MAXPROC);
}

// Sporks the daemons (device drivers) for Phase 4
void phase4_start_service_processes()
{
    spork("Sleep Daemon", sleep_daemon_process, NULL, USLOSS_MIN_STACK, 1);

    spork("Terminal 0 Daemon Process", terminal_daemon_process, (void*)0, USLOSS_MIN_STACK, 1);
    spork("Terminal 1 Daemon Process", terminal_daemon_process, (void*)1, USLOSS_MIN_STACK, 1);
    spork("Terminal 2 Daemon Process", terminal_daemon_process, (void*)2, USLOSS_MIN_STACK, 1);
    spork("Terminal 3 Daemon Process", terminal_daemon_process, (void*)3, USLOSS_MIN_STACK, 1);

    spork("Disk 0 Daemon Process", disk_daemon_process, (void*)0, USLOSS_MIN_STACK, 1);
    spork("Disk 1 Daemon Process", disk_daemon_process, (void*)1, USLOSS_MIN_STACK, 1);
}