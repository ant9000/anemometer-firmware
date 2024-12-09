#include <string.h>
#include <assert.h>
#include "periph/flashpage.h"

#define RWWEE_PAGE 0
#define SIGNATURE "RIOT"
#define SIGNATURE_LEN 4

int load_from_nvm (void *buffer, size_t len)
{
    uint8_t page[FLASHPAGE_SIZE];
    if (len + SIGNATURE_LEN > FLASHPAGE_SIZE)
        return -2;
    flashpage_rwwee_read(RWWEE_PAGE, page);
    if (strncmp((char *)page, SIGNATURE, SIGNATURE_LEN) != 0)
        return -1;
    memcpy((uint8_t *)buffer, page + SIGNATURE_LEN, len);
    return 0;
}

int save_to_nvm (const void *buffer, size_t len)
{
    uint8_t page[FLASHPAGE_SIZE];
    if (len + SIGNATURE_LEN > FLASHPAGE_SIZE)
        return -2;
    // do not rewrite emulated EEPROM unless needed
    flashpage_rwwee_read(RWWEE_PAGE, page);
    if (
        (strncmp((char *)page, SIGNATURE, SIGNATURE_LEN) != 0) ||
        (memcmp(buffer, page + SIGNATURE_LEN, len) != 0)
    ) {
        memcpy(page, SIGNATURE, SIGNATURE_LEN);
        memcpy(page + SIGNATURE_LEN, buffer, len);
        if (flashpage_rwwee_write_and_verify(RWWEE_PAGE, page) != FLASHPAGE_OK) { return -1; }
        return 1;
    }
    return 0;
}

int erase_nvm (void)
{
    uint8_t page[FLASHPAGE_SIZE];
    memset(page, 0, FLASHPAGE_SIZE);
    if (flashpage_rwwee_write_and_verify(RWWEE_PAGE, page) != FLASHPAGE_OK) { return -1; }
    return 0;
}
