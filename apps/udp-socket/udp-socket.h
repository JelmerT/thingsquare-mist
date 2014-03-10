#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include "net/uip.h"

struct udp_socket;

typedef void (* udp_socket_input_callback_t)(struct udp_socket *c,
                                             void *ptr,
                                             const uip_ipaddr_t *source_addr,
                                             uint16_t source_port,
                                             const uip_ipaddr_t *dest_addr,
                                             uint16_t dest_port,
                                             const uint8_t *data,
                                             uint16_t datalen);

struct udp_socket {
  struct udp_socket *next;

  udp_socket_input_callback_t input_callback;
  void *ptr;

  struct process *p;

  struct uip_udp_conn *udp_conn;

};

int udp_socket_register(struct udp_socket *c,
                        void *ptr,
                        udp_socket_input_callback_t receive_callback);

int udp_socket_bind(struct udp_socket *c,
                    uint16_t local_port);

int udp_socket_connect(struct udp_socket *c,
                       uip_ipaddr_t *remote_addr,
                       uint16_t remote_port);

int udp_socket_send(struct udp_socket *c,
                    const void *data, uint16_t datalen);

int udp_socket_sendto(struct udp_socket *c,
                      const void *data, uint16_t datalen,
                      const uip_ipaddr_t *addr, uint16_t port);

void udp_socket_init(void);

#endif /* UDP_SOCKET_H */
