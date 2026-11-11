#ifndef UROS_TRANSPORT_H
#define UROS_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

/* 4 ham transport cho micro-XRCE-DDS. Muon doi sang USB CDC chi can viet lai 4 ham nay. */
bool   uros_transport_open (struct uxrCustomTransport *transport);
bool   uros_transport_close(struct uxrCustomTransport *transport);
size_t uros_transport_write(struct uxrCustomTransport *transport,
                            const uint8_t *buf, size_t len, uint8_t *err);
size_t uros_transport_read (struct uxrCustomTransport *transport,
                            uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif /* UROS_TRANSPORT_H */
