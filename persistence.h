#ifndef PERSISTENCE_H
#define PERSISTENCE_H

int load_from_nvm (void *buffer, size_t len);
int save_to_nvm (const void *buffer, size_t len);
int erase_nvm (void);

#endif /* PERSISTENCE_H */
