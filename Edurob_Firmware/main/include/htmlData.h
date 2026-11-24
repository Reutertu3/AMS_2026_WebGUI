#ifndef HTMLDATA_H
#define HTMLDATA_H

const static char sse_resp[] = "HTTP/1.1 200 OK\r\n"
                               "Cache-Control: no-store\r\n"
                               "Content-Type: text/event-stream\r\n"
                               "\r\n"
                               "retry: 20000\r\n"
                               "\r\n";

#endif /* HTMLDATA_H */
