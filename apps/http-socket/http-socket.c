/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "mist.h"
#include "http-socket.h"

#define MAX_PATHLEN 80
#define MAX_HOSTLEN 40
PROCESS(http_socket_process, "HTTP socket process");
LIST(socketlist);
/*---------------------------------------------------------------------------*/
static void
call_callback(struct http_socket *s, http_socket_event_t e,
              const uint8_t *data, uint16_t datalen)
{
  if(s->callback != NULL) {
    s->callback(s, s->callbackptr, e,
                data, datalen);
  }
}
/*---------------------------------------------------------------------------*/
static void
parse_header_init(struct http_socket *s)
{
  PT_INIT(&s->headerpt);
}
/*---------------------------------------------------------------------------*/
static int
parse_header_byte(struct http_socket *s, char c)
{
  static int i;
  static char status_code[3];
  static char ending[4];
  PT_BEGIN(&s->headerpt);

  /* Skip the HTTP response */
  while(c != ' ') {
    PT_YIELD(&s->headerpt);
  }

  /* Skip the space */
  PT_YIELD(&s->headerpt);
  /* Read three bytes of HTTP status */
  for(i = 0; i < 3; i++) {
    status_code[i] = c;
    PT_YIELD(&s->headerpt);
  }

  if(memcmp(status_code, "404", 3) == 0) {
    printf("File not found\n");
    call_callback(s, HTTP_SOCKET_404, NULL, 0);
    while(1) {
      PT_YIELD(&s->headerpt);
    }
  } else if(memcmp(status_code, "301", 3) == 0 ||
            memcmp(status_code, "302", 3) == 0) {
    printf("File moved (not handled)\n");
    while(1) {
      PT_YIELD(&s->headerpt);
    }
  } else if(memcmp(status_code, "200", 3) == 0) {
    /* Skip headers until data */

    do {
      /* Read bytes until first \r */
      do {
        PT_YIELD(&s->headerpt);
      } while(c != '\r');

      /* Check for \r\n\r\n, otherwise continue looping */
      i = 0;
      while((c == '\r' || c == '\n') && i < sizeof(ending)) {
        ending[i] = c;
        i++;
        PT_YIELD(&s->headerpt);
      }
    } while(i != 4 && memcmp(ending, "\r\n\r\n", 4) != 0);

    /* All headers skipped, now read data */
    /* Should exit the pt here to indicate that all headers have been
       read */
    PT_EXIT(&s->headerpt);
  } else {
    while(1) {
      PT_YIELD(&s->headerpt);
    }
  }

  PT_END(&s->headerpt);
}
/*---------------------------------------------------------------------------*/
static int
input_pt(struct http_socket *s,
         const uint8_t *inputptr, int inputdatalen)
{
  int i;
  PT_BEGIN(&s->pt);

  /* Parse the header */
  s->header_received = 0;
  do {
    for(i = 0; i < inputdatalen; i++) {
      if(!PT_SCHEDULE(parse_header_byte(s, inputptr[i]))) {
        s->header_received = 1;
        break;
      }
    }
    inputdatalen -= i;
    inputptr += i;

    if(s->header_received == 0) {
      /* If we have not yet received the full header, we wait for the
         next packet to arrive. */
      PT_YIELD(&s->pt);
    }
  } while(s->header_received == 0);

  do {
    /* Receive the data */
    call_callback(s, HTTP_SOCKET_DATA, inputptr, inputdatalen);
    PT_YIELD(&s->pt);
  } while(inputdatalen > 0);

  PT_END(&s->pt);
}
/*---------------------------------------------------------------------------*/
static int
input(struct tcp_socket *tcps, void *ptr,
      const uint8_t *inputptr, int inputdatalen)
{
  struct http_socket *s = ptr;

  input_pt(s, inputptr, inputdatalen);

  return 0; /* all data consumed */
}
/*---------------------------------------------------------------------------*/
static int
parse_url(const char *url, char *host, uint16_t *portptr, char *path)
{
  const char *urlptr;
  int i;
  const char *file;
  uint16_t port;

  if(url == NULL) {
    printf("null url");
    return 0;
  }

  /* Don't even try to go further if the URL is empty. */
  if(strlen(url) == 0) {
    printf("empty url");
    return 0;
  }

  /* See if the URL starts with http:// and remove it. Otherwise, we
     assume it is an implicit http://. */
  if(strncmp(url, "http://", strlen("http://")) == 0) {
    urlptr = url + strlen(http_http);
  } else {
    urlptr = url;
  }

  /* Find host part of the URL. */
  if(*urlptr == '[') {
    /* Handle IPv6 addresses - scan for matching ']' */
    urlptr++;
    for(i = 0; i < MAX_HOSTLEN; ++i) {
      if(*urlptr == ']') {
        if(host != NULL) {
          host[i] = 0;
        }
        break;
      }
      if(host != NULL) {
        host[i] = *urlptr;
      }
      ++urlptr;
    }
  } else {
    for(i = 0; i < MAX_HOSTLEN; ++i) {
      if(*urlptr == 0 ||
         *urlptr == '/' ||
         *urlptr == ' ' ||
         *urlptr == ':') {
        if(host != NULL) {
          host[i] = 0;
      }
        break;
      }
      if(host != NULL) {
        host[i] = *urlptr;
      }
      ++urlptr;
    }
  }

  /* Find the port. Default is 80. */
  port = 80;
  if(*urlptr == ':') {
    port = 0;
    do {
      ++urlptr;
      if(*urlptr >= '0' && *urlptr <= '9') {
	port = (10 * port) + (*urlptr - '0');
      }
    } while(*urlptr >= '0' &&
	    *urlptr <= '9');
  }
  if(portptr != NULL) {
    *portptr = port;
  }
  /* Find file part of the URL. */
  while(*urlptr != '/' && *urlptr != 0) {
    ++urlptr;
  }
  if(*urlptr == '/') {
    file = urlptr;
  } else {
    file = "/";
  }
  if(path != NULL) {
    strncpy(path, file, MAX_PATHLEN);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static void
removesocket(struct http_socket *s)
{
  list_remove(socketlist, s);
}
/*---------------------------------------------------------------------------*/
static void
event(struct tcp_socket *tcps, void *ptr,
      tcp_socket_event_t e)
{
  struct http_socket *s = ptr;
  char host[MAX_HOSTLEN];
  char path[MAX_PATHLEN];
  uint16_t port;


  if(e == TCP_SOCKET_CONNECTED) {
    printf("Connected\n");
    if(parse_url(s->url, host, &port, path)) {
      tcp_socket_send_str(tcps, "GET ");
      tcp_socket_send_str(tcps, path);
      tcp_socket_send_str(tcps, " HTTP/1.0\r\n");
      tcp_socket_send_str(tcps, "Host: ");
      tcp_socket_send_str(tcps, host);
      tcp_socket_send_str(tcps, "\r\n");
      tcp_socket_send_str(tcps, "\r\n");
    }
    parse_header_init(s);
  } else if(e == TCP_SOCKET_CLOSED) {
    call_callback(s, HTTP_SOCKET_CLOSED, NULL, 0);
    removesocket(s);
    printf("Closed\n");
  } else if(e == TCP_SOCKET_TIMEDOUT) {
    call_callback(s, HTTP_SOCKET_TIMEDOUT, NULL, 0);
    removesocket(s);
    printf("Timedout\n");
  } else if(e == TCP_SOCKET_ABORTED) {
    call_callback(s, HTTP_SOCKET_ABORTED, NULL, 0);
    removesocket(s);
    printf("Aborted\n");
  }
}
/*---------------------------------------------------------------------------*/
static int
start_get(struct http_socket *s)
{
  uip_ip4addr_t ip4addr;
  uip_ip6addr_t ip6addr;
  uip_ip6addr_t *addr;
  char host[MAX_HOSTLEN];
  char path[MAX_PATHLEN];
  uint16_t port;

  if(parse_url(s->url, host, &port, path)) {

    printf("url %s host %s port %d path %s\n",
           s->url, host, port, path);

    /* First check if the host is an IP address. */
    if(uiplib_ip6addrconv(host, &ip6addr) == 0) {
      if(uiplib_ip4addrconv(host, &ip4addr) != 0) {
        ip64_addr_4to6(&ip4addr, &ip6addr);
      } else {
        /* Try to lookup the hostname. If it fails, we initiate a hostname
           lookup. */
        addr = mdns_lookup(host);
        if(addr == NULL) {
          mdns_query(host);
          printf("Resolving host...\n");
          return HTTP_SOCKET_OK;
        }
        tcp_socket_connect(&s->s, addr, port);
        return HTTP_SOCKET_OK;
      }
    }
    tcp_socket_connect(&s->s, &ip6addr, port);
    return HTTP_SOCKET_OK;
  } else {
    return HTTP_SOCKET_ERR;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(http_socket_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT();

    if(ev == mdns_event_found && data != NULL) {
      struct http_socket *s;
      const char *name = data;
      /* Either found a hostname, or not. We need to go through the
	 list of http sockets and figure out to which connection this
	 reply corresponds, then either restart the HTTP get, or kill
	 it (if no hostname was found). */
      for(s = list_head(socketlist);
	  s != NULL;
	  s = list_item_next(s)) {
	char host[MAX_HOSTLEN];
	if(parse_url(s->url, host, NULL, NULL) &&
	   strcmp(name, host) == 0) {
	  if(mdns_lookup(name) != NULL) {
	    /* Hostname found, restart get. */
            start_get(s);
	  } else {
	    /* Hostname not found, kill connection. */
            call_callback(s, HTTP_SOCKET_HOSTNAME_NOT_FOUND, NULL, 0);
            removesocket(s);
	  }
	}
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  static uint8_t inited = 0;
  if(inited == 0) {
    process_start(&http_socket_process, NULL);
    list_init(socketlist);
    inited = 1;
  }
}
/*---------------------------------------------------------------------------*/
int
http_socket_get(struct http_socket *s,
                const char *url,
                http_socket_callback_t callback,
                void *callbackptr)
{
  init();

  strncpy(s->url, url, sizeof(s->url));
  tcp_socket_register(&s->s, s,
                      s->inputbuf, sizeof(s->inputbuf),
                      s->outputbuf, sizeof(s->outputbuf),
                      input, event);

  s->callback = callback;
  s->callbackptr = callbackptr;

  list_add(socketlist, s);

  return start_get(s);
}
/*---------------------------------------------------------------------------*/
