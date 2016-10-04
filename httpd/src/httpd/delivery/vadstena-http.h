/**
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * \copyright 2011-2016 Melown Technologies SE
 * \copyright Strakonicka 1199/2d, 150 00 Praha 5, Czech Republic
 * \copyright All rights reserved.
 *
 * This is a proprietary software. To be used only with valid licence.
 * Not to be redistributed.
 * Scope of the licence is subject to the license agreement.
 *
 * \file vadstena-http.h
 * Vadstena tileset HTTP library interface.
 */

#ifndef vadstena_vadstena_http_http_hpp_included_
#define vadstena_vadstena_http_http_hpp_included_

#include <stdlib.h>
#include <time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    TILESET_OK               /** No error occurred */
    , TILESET_UNINITIALIZED  /** Tile set not found */
    , TILESET_NOTFOUND       /** Tile set not found */
    , TILESET_FILE_NOTFOUND  /** File not found */
    , TILESET_MUST_REDIRECT  /** Tile set was found but slash must be appended
                                 for proper reference */
    , TILESET_FAILED         /** Generic error */
    , TILESET_ERRNO          /** Inspect errno for error */
};

enum {
    /** Disable browser. */
    TILESET_OPEN_DISABLE_BROWSER       = 0x0001
    /** Enables TS to VTS0 adapter. */
    , TILESET_OPEN_ENABLE_VTS0_ADAPTER = 0x0002
};

/** File type definition.
 */
typedef enum {
    TILESET_FILETYPE_TILE       /** Tile data. */
    , TILESET_FILETYPE_FILE     /** Other tileset file */
    , TILESET_FILETYPE_BROWSER  /** Browser file */
    , TILESET_FILETYPE_HOT      /** Hot file - not to be cached */
} tileset_FileType;

/** Handle to open tile set file.
 */
typedef void tileset_File;

/** File statistics.
 */
typedef struct {
    /** Size of file */
    size_t size;

    /** Timestamp of last modification to the file. */
    time_t lastModified;

    /** Content type of file (static string). */
    const char *contentType;

    /** File type. */
    tileset_FileType fileType;
} tileset_Stat;

/** Read-only open file info.
 */
typedef struct {
    /** File descriptor. Value < 0 means no open file is available. */
    int fd;

    /** File offset of the start of data of interest. */
    size_t start;

    /** File offset of the end of data of interest. */
    size_t end;

    /** File offset of the end of data of interest. */
    int shared;
} tileset_OpenFile;

/** Read-only memory block.
 */
typedef struct {
    /** Pointer to read only data */
    const void *data;

    /** File offset of the end of data of interest. */
    size_t size;
} tileset_Memory;

/** Variables passed to open handler.
 *  Opaque structure.
 */
typedef struct tileset_Variables tileset_Variables;

/** Returns last error that occurred in this thread.
 */
extern int tileset_errno();

/** Returns last error message. Valid only when tileset_errno() != TILESET_OK
 */
extern const char* tileset_errorMessage();

/** Initialize the HTTP library.
 *
 * \param threadSafe make library thread safe
 */
extern void tileset_initialize(int threadSafe);

/** Deinitialize the HTTP library.
 *  Library is automatically deinitialized on exit.
 */
extern void tileset_finish();

/** Run cleanup code. Removes unused stuff.
 *
 * NB: cleanup is called internally on every tileset_open()
 *
 * \return 0 = success, -1 = failure
 */
extern int tileset_cleanup();

/** Load registry from given path.
 *
 * VTS does not run without loaded registry.
 *
 * \param registry path to registry or NULL to use default path
 * \return 0 = success, -1 = failure
 */
extern int tileset_registry(const char *registry);

/** Sets dbglog logging mask.
 *
 * \param mask mask
 * \param length of mask or -1 if mask is NULL terminated
 * \return 0 = success, -1 = failure
 */
extern int tileset_logMask(const char *mask, int len);

/** Sets dbglog log file.
 *
 * \param filename path to file
 * \param length of filename or -1 if mask is NULL terminated
 * \return 0 = success, -1 = failure
 */
extern int tileset_logFile(const char *filename, int len);

/** Sets dbglog log file.
 *
 * \param enabled dblog logs to console (stderr) when non-zero
 * \return 0 = success, -1 = failure
 */
extern int tileset_logConsole(int enabled);

/** Returns handle to file inside tile set.
 *
 *  You have to call tileset_close function to claim back all resources
 *  associated with the returned file.
 *
 *  In case of failure, returns NULL and sets tileset_errno appropriately.
 *
 *  \param path path to file in format /path/to/tileset/FILENAME
 *  \param flags open flags, 0 or or-ed TILESET_OPEN_* constants
 *  \return open file or NULL on failure
 */
extern tileset_File* tileset_open(const char *path, int flags);

/** Returns handle to file inside tile set.
 *
 *  You have to call tileset_close function to claim back all resources
 *  associated with the returned file.
 *
 *  In case of failure, returns NULL and sets tileset_errno appropriately.
 *
 *  Variables pointer can be NULL.
 *
 *  \param path path to file in format /path/to/tileset/FILENAME
 *  \param flags open flags, 0 or or-ed TILESET_OPEN_* constants
 *  \param variables variables passed to open handler
 *  \return open file or NULL
 */
extern tileset_File* tileset_open2(const char *path, int flags
                                   , const tileset_Variables *variables);

/** Closes a tile set file handle.
 *
 *  In case of failure, returns -1 and sets tileset_errno appropriately.
 *
 *  \param file pointer to file obtained by calling tileset_open()
 *  \return 0 = success, -1 = failure
 */
extern int tileset_close(tileset_File *file);

/** Generic read interface. Reads data from tile set file.
 *  Returns 0 on EOF
 *
 *  In case of failure, returns -1 and sets tileset_errno appropriately.
 *
 *  NB: You can use tileset_getOpenFile or tileset_getMemory to obtain
 *  data as open file descriptor or memory block if available.
 *
 *  \param file pointer to file obtained by calling tileset_open()
 *  \param buf pointer to memory buffer
 *  \param size number of bytes available in buffer
 *  \return >0 number of read bytes, 0 = EOF, -1 = failure
 */
extern int tileset_read(tileset_File *file, void *buf, size_t size);

/** Return tileset file statistics.
 *  In case of failure, returns -1 and sets tileset_errno appropriately.
 *
 *  \param file pointer to file obtained by calling tileset_open()
 *  \param stat pointer to stat structure
 *  \return 0 = success, -1 = failure
 */
extern int tileset_stat(tileset_File *file, tileset_Stat *stat);

/** Returns open file assodicated with this tileset.
 *  Sets openFile->fd to -1 if there is no associated file.
 *  Precautions: if openFile->shared is non-zero then:
 *     * never close fd, it is owned by associated tileset_File
 *     * never change file offset of fd, i.e. no lseek, no read
 *       -> use pread or aio
 *     * fstat(fd) can report other size st_size than (end - start)
 *
 *  In case of failure, returns -1 and sets tileset_errno appropriately.
 *
 *  \param file pointer to file obtained by calling tileset_open()
 *  \param stat pointer to open file structure
 *  \return 0 = success, -1 = failure
 */
extern int tileset_getOpenFile(tileset_File *file
                               , tileset_OpenFile *openFile);

/** Returns memory with file content.
 *  Sets memory->data to NULL if there is no associated memory block.
 *
 *  In case of failure, returns -1 and sets tileset_errno appropriately.
 *
 *  \param file pointer to file obtained by calling tileset_open()
 *  \param memory pointer to memory structure
 *  \return 0 = success, -1 = failure
 */
extern int tileset_getMemory(tileset_File *file
                             , tileset_Memory *memory);

/** Creates new variable table.
 *
 *  Table must be destroyes by call to tileset_destroyVariables.
 *
 *  \return variables table or NULL on failure
 */
extern tileset_Variables* tileset_createVariables();

/** Destroys variables table.
 *
 *  Table must be pointer obtained by calling tileset_createVariables.
 *
 *  \param variables variables to destroy
 *  \return 0 = success, -1 = failure
 */
extern int tileset_destroyVariables(tileset_Variables *variables);

/** Clones existing variable table.
 *
 *  Table must be destroyes by call to tileset_destroyVariables.
 *
 *  \param variables variables table to clone
 *  \return variables table or NULL on failure
 */
extern tileset_Variables*
tileset_cloneVariables(const tileset_Variables *variables);

/** Merges variables from src into dst.
 *
 *  \param dst destination variables
 *  \param src source variables
 *  \param overwrite overwrite existing values in dst if nonzero
 *  \return variables table or NULL on failure
 */
extern int tileset_mergeVariables(tileset_Variables *dst
                                  , const tileset_Variables *src
                                  , int overwrite);

/** Adds new entry into table. Existing is rewritten (if present).
 *
 *  \param variables variables table to update
 *  \param key variable name
 *  \param keySize size of variable name
 *  \param value variable value
 *  \param valueSize size of variable value
 *  \param overwrite overwrite existing values in variables if nonzero
 *  \return 0 = success, -1 = failure
 */
extern int tileset_addVariable(tileset_Variables *variables
                               , const char *key, size_t keySize
                               , const char *value, size_t valueSize
                               , int overwrite);

#ifdef __cplusplus
}
#endif

#endif /** vadstena_vadstena_http_http_hpp_included_ */
