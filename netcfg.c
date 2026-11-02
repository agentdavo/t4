#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "netcfg.h"

extern int verbose;
extern int shlinks;

#define NODE_HOST          -1
#define NODE_NOTCONNECTED  -2
#define NODE_INVALID       -3
#define NODE_IPC           -4
#define NODE_SHM           -5

typedef struct _NETURL {
        int  node, link;
        char *url;
} NETURL;

#define MAX_NETURL 4096
NETURL NetURLs[MAX_NETURL];
int nNetURL;

typedef struct _NETLINK {
        int node, link;
        int othernode, otherlink;
        int router, otherrouter;        /* named "router" in the map */
} NETLINK;

#define MAX_NETLINK  MAX_NETURL
NETLINK NetLinks[MAX_NETLINK];
int nNetLink;


static int network, address, port, scale;
static int max_nodeid;

static char* ip2str (int a, char *tmp)
{
        int cls[4];

        cls[3] = a & 255; a >>= 8;
        cls[2] = a & 255; a >>= 8;
        cls[1] = a & 255; a >>= 8;
        cls[0] = a & 255;
        sprintf (tmp, "%d.%d.%d.%d", cls[0], cls[1], cls[2], cls[3]);
        return tmp;
}

static void dump_netparams (void)
{
        char tmpstr[32];
        int a;

        if (!verbose)
                return;

        a = network ? network : address;
        printf ("-I-EMU414: %s = %s Port = %d Scale = %d\n",
                network ? "Network" : "Address",
                ip2str (a, tmpstr),
                port,
                scale);
}

static char* tcpLinkURL (int node, int link)
{
        static char buf[128];
        char tmpstr[32];
        int a, p;

        a = network ? network + node : address;
        p = address ? port + node * scale : port;
        p += link;

        sprintf (buf, "tcp://%s:%d", ip2str (a, tmpstr), p);
        return &buf[0];
}

static char* skip (char *ptr)
{
        while (*ptr && isspace (*ptr))
                ptr++;
        return ptr;
}

static char* read_number (char *ptr, int *node)
{
        int n, m, ch, hasdot;

        *node = NODE_INVALID;
        ptr = skip (ptr);
        if (0 == *ptr)
                return ptr;

        /* fprintf (stderr, "read_number = [%s]\n", ptr); */
        n = 0; m = 0; hasdot = 0;
        while (isdigit(*ptr) || '.' == *ptr)
        {
                ch = *ptr;
                if ('.' == ch)
                {
                        n = (n << 8) + m; m = 0;
                        hasdot = 1;
                }
                else
                        m = 10*m + ch - '0';
                ptr++;
        }
        n = hasdot ? (n << 8) + m : m;
        *node = n;
        return ptr;
}

static char* read_string (char *ptr, char *buf)
{
        buf[0] = '\0';
        ptr = skip (ptr);
        if (0 == *ptr)
                return ptr;
        while (!isspace (*ptr))
                *buf++ = *ptr++;
        *buf++ = '\0';
        return ptr;
}

/* Whether the last link read_link() parsed was named "router N link L". */
static int link_router;

static char* read_link (char *ptr, int *node, int *link)
{
        link_router = 0;
        ptr = skip (ptr);
        /* fprintf (stderr, "read_link = [%s]\n", ptr); */
        if (0 == *ptr)
        {
                *node = NODE_NOTCONNECTED;
                return ptr;
        }
        else if ((0 == strncmp (ptr, "HOST", 4)) ||
                 (0 == strncmp (ptr, "host", 4)))
        {
                *node = NODE_HOST;
                return ptr + 4;
        }
        else if ((0 == strncmp (ptr, "SHM", 3)) ||
                 (0 == strncmp (ptr, "shm", 3)))
        {
                *node = NODE_SHM;
                return ptr + 3;
        }
        else if ((0 == strncmp (ptr, "IPC", 3)) ||
                 (0 == strncmp (ptr, "ipc", 3)))
        {
                *node = NODE_IPC;
                return ptr + 3;
        }
        else if (0 == strncmp (ptr, "processor ", 10) ||
                 0 == strncmp (ptr, "router ", 7))
        {
                int router = 'r' == *ptr;

                link_router = router;
                /* "processor"|"router" <number> "link" <number> */
                ptr = read_number (ptr + (router ? 7 : 10), node);
                if (NODE_INVALID == *node)
                        return ptr;
                ptr = skip (ptr);
                if (0 == *ptr)
                {
                        *node = NODE_INVALID;
                        return ptr;
                }
                /* a router's links 4-5 are data links, and its control
                 * link is not modelled */
                if (!router && 0 == strncmp (ptr, "clink ", 6))
                {
                        /* T9000 control link C = link 4 + C (t9000_ctl.c) */
                        ptr = read_number (ptr + 6, link);
                        if (*link < 0 || *link > 1)
                                *node = NODE_INVALID;
                        else
                                *link += 4;
                        return ptr;
                }
                if (strncmp (ptr, "link ", 5))
                {
                        *node = NODE_INVALID;
                        return ptr;
                }
                ptr = read_number (ptr + 5, link);
                if (NODE_INVALID == *link)
                        *node = NODE_INVALID;
                return ptr;
        }
        else if (0 == strncmp (ptr, "-", 1))
        {
                *node = NODE_NOTCONNECTED;
                return ptr + 1;
        }
        ptr = read_number (ptr, node);
        if (NODE_INVALID == *node)
                return ptr;
        if (strncmp (ptr, "-", 1))
        {
                /* fprintf (stderr, "XXX [%s]\n", ptr); */
                return ptr + 1;
        }
        ptr = read_number (ptr + 1, link);
        if (NODE_INVALID == *link)
                *node = NODE_INVALID;
        return ptr;
}


void add_url (int node, int link, char *url)
{
        if (verbose)
                printf ("-I-EMU414: Node %d Link%dIn at %s\n", node, link, url);
        NetURLs[nNetURL].node = node;
        NetURLs[nNetURL].link = link;
        NetURLs[nNetURL++].url = strdup (url);
}

/* ---- IMS C104 routers ---------------------------------------------------
 *
 * "router" <id>                              declare a C104 (32 links)
 * "router" <id> "links" <n>                  ... with n links
 * "router" <id> "header" 1|2 [on <links>]    input header length (1)
 * "router" <id> "interval" <lo> <hi> <out> [on <links>]
 *                                            lo <= header < hi leaves on
 *                                            link <out>, or on the first
 *                                            free link of a group a-b
 * "router" <id> "delete" <links>             header deletion on outputs
 *
 * <links> is a comma-separated list of n or a-b; without "on" a statement
 * applies to every input link.  Numbers are decimal, #hex or 0xhex. */

static net_router_t *Routers;
static int nRouters;

int netRouterCount (void)
{
        return nRouters;
}

net_router_t* netRouter (int i)
{
        return (i >= 0 && i < nRouters) ? &Routers[i] : NULL;
}

net_router_t* netRouterById (int id)
{
        int i;

        for (i = 0; i < nRouters; i++)
                if (Routers[i].id == id)
                        return &Routers[i];
        return NULL;
}

static net_router_t* add_router (int id)
{
        net_router_t *r = netRouterById (id);

        if (r)
                return r;
        r = realloc (Routers, (nRouters + 1) * sizeof (*r));
        if (NULL == r)
                return NULL;
        Routers = r;
        r = &Routers[nRouters++];
        memset (r, 0, sizeof (*r));
        r->id = id;
        r->nlinks = NET_ROUTER_LINKS;
        return r;
}

static char* read_uint (char *ptr, unsigned *val, int *ok)
{
        unsigned v = 0, base = 10;
        int n = 0;

        ptr = skip (ptr);
        if ('#' == *ptr)
        {
                base = 16; ptr++;
        }
        else if ('0' == ptr[0] && ('x' == ptr[1] || 'X' == ptr[1]))
        {
                base = 16; ptr += 2;
        }
        for (;; ptr++, n++)
        {
                if (isdigit (*ptr))
                        v = v * base + (*ptr - '0');
                else if (16 == base && isxdigit (*ptr))
                        v = v * base + (tolower (*ptr) - 'a' + 10);
                else
                        break;
        }
        *val = v;
        *ok = n > 0;
        return ptr;
}

/* <link> | <link>-<link>; *first < 0 if malformed */
static char* read_range (char *ptr, int *first, int *last)
{
        unsigned a, b;
        int ok;

        *first = -1;
        ptr = read_uint (ptr, &a, &ok);
        if (!ok)
                return ptr;
        b = a;
        if ('-' == *ptr && isdigit (ptr[1]))
        {
                ptr = read_uint (ptr + 1, &b, &ok);
                if (!ok)
                        return ptr;
        }
        if (a > b || b >= NET_ROUTER_LINKS)
                return ptr;
        *first = a;
        *last = b;
        return ptr;
}

/* <range> [, <range>]...; *mask = 0 if malformed */
static char* read_linkset (char *ptr, unsigned *mask)
{
        int a, b;

        *mask = 0;
        for (;;)
        {
                ptr = read_range (ptr, &a, &b);
                if (a < 0)
                {
                        *mask = 0;
                        return ptr;
                }
                for (; a <= b; a++)
                        *mask |= 1u << a;
                ptr = skip (ptr);
                if (',' != *ptr)
                        return ptr;
                ptr++;
        }
}

/* [ "on" <links> ]: every link when absent */
static char* read_on (char *ptr, unsigned *mask)
{
        ptr = skip (ptr);
        if (0 == strncmp (ptr, "on", 2) && isspace (ptr[2]))
                return read_linkset (ptr + 2, mask);
        *mask = ~0u;
        return ptr;
}

static int read_router (char *ptr, int nline)
{
        net_router_t *r;
        net_interval_t *it;
        unsigned id, v, lo, hi, on;
        int ok, a, b;

        ptr = read_uint (ptr, &id, &ok);
        if (!ok)
        {
                printf ("-E-EMU414: Missing router id at line %d.\n", nline);
                return (-1);
        }
        if (NULL == (r = add_router ((int)id)))
        {
                printf ("-E-EMU414: Out of memory at line %d.\n", nline);
                return (-1);
        }
        ptr = skip (ptr);
        if (0 == *ptr || 0 == strncmp (ptr, "--", 2))
                return 0;
        if (0 == strncmp (ptr, "links", 5))
        {
                ptr = read_uint (ptr + 5, &v, &ok);
                if (!ok || v < 1 || v > NET_ROUTER_LINKS)
                {
                        printf ("-E-EMU414: Router link count must be 1-%d at line %d.\n",
                                NET_ROUTER_LINKS, nline);
                        return (-1);
                }
                r->nlinks = v;
        }
        else if (0 == strncmp (ptr, "header", 6))
        {
                ptr = read_uint (ptr + 6, &v, &ok);
                if (!ok || (1 != v && 2 != v))
                {
                        printf ("-E-EMU414: Header length must be 1 or 2 at line %d.\n", nline);
                        return (-1);
                }
                ptr = read_on (ptr, &on);
                if (0 == on)
                        goto BadLinks;
                if (2 == v)
                        r->hdr2 |= on;
                else
                        r->hdr2 &= ~on;
        }
        else if (0 == strncmp (ptr, "interval", 8))
        {
                ptr = read_uint (ptr + 8, &lo, &ok);
                if (ok)
                        ptr = read_uint (ptr, &hi, &ok);
                if (!ok || hi <= lo || hi > 0x10000)
                {
                        printf ("-E-EMU414: Interval needs 0 <= lo < hi <= #10000 at line %d.\n", nline);
                        return (-1);
                }
                ptr = read_range (ptr, &a, &b);
                if (a < 0)
                        goto BadLinks;
                ptr = read_on (ptr, &on);
                if (0 == on)
                        goto BadLinks;
                it = realloc (r->ints, (r->nint + 1) * sizeof (*it));
                if (NULL == it)
                {
                        printf ("-E-EMU414: Out of memory at line %d.\n", nline);
                        return (-1);
                }
                r->ints = it;
                it += r->nint++;
                it->lo = lo;
                it->hi = hi;
                it->first = a;
                it->last = b;
                it->on = on;
        }
        else if (0 == strncmp (ptr, "delete", 6))
        {
                ptr = read_linkset (ptr + 6, &on);
                if (0 == on)
                        goto BadLinks;
                r->del |= on;
        }
        else
        {
                printf ("-E-EMU414: Unknown router statement at line %d.\n", nline);
                return (-1);
        }
        ptr = skip (ptr);
        if (*ptr && strncmp (ptr, "--", 2))
        {
                printf ("-E-EMU414: Unexpected '%.*s' at line %d.\n",
                        (int)strcspn (ptr, " \t\r\n"), ptr, nline);
                return (-1);
        }
        return 0;
BadLinks:
        printf ("-E-EMU414: Bad router link list at line %d.\n", nline);
        return (-1);
}

/* Interval selectors must not overlap (HRM 7.3.1) and must name links the
 * router has. */
/* Whether a node named in a Connect line is what the map declares it to be. */
static int check_node_kind (int node, int router)
{
        if (node < 0 || router == (NULL != netRouterById (node)))
                return 0;
        if (router)
                printf ("-E-EMU414: Router %d is connected but not declared ('router %d').\n",
                        node, node);
        else
                printf ("-E-EMU414: Node %d is a router; connect it as 'router %d link L'.\n",
                        node, node);
        return (-1);
}

static int check_routers (void)
{
        int i, j, k;

        for (j = 0; j < nNetLink; j++)
                if (check_node_kind (NetLinks[j].node, NetLinks[j].router) < 0 ||
                    check_node_kind (NetLinks[j].othernode, NetLinks[j].otherrouter) < 0)
                        return (-1);
        for (i = 0; i < nRouters; i++)
        {
                net_router_t *r = &Routers[i];
                unsigned all = r->nlinks >= 32 ? ~0u : (1u << r->nlinks) - 1;

                r->hdr2 &= all;
                r->del &= all;
                for (j = 0; j < r->nint; j++)
                {
                        net_interval_t *a = &r->ints[j];

                        a->on &= all;
                        if (a->last >= r->nlinks)
                        {
                                printf ("-E-EMU414: Router %d has no Link%d.\n", r->id, a->last);
                                return (-1);
                        }
                        for (k = 0; k < j; k++)
                        {
                                net_interval_t *b = &r->ints[k];

                                if ((a->on & b->on) && a->lo < b->hi && b->lo < a->hi)
                                {
                                        printf ("-E-EMU414: Router %d: intervals [#%X,#%X) and "
                                                "[#%X,#%X) overlap.\n",
                                                r->id, b->lo, b->hi, a->lo, a->hi);
                                        return (-1);
                                }
                        }
                }
                for (j = 0; j < nNetLink; j++)
                {
                        if ((NetLinks[j].node == r->id && NetLinks[j].link >= r->nlinks) ||
                            (NetLinks[j].othernode == r->id && NetLinks[j].otherlink >= r->nlinks))
                        {
                                printf ("-E-EMU414: Router %d has no Link%d.\n", r->id,
                                        NetLinks[j].node == r->id ? NetLinks[j].link
                                                                  : NetLinks[j].otherlink);
                                return (-1);
                        }
                }
                if (verbose)
                        printf ("-I-EMU414: Router %d: %d links, %d intervals\n",
                                r->id, r->nlinks, r->nint);
        }
        return 0;
}

static int add_netlink (int node, int link, int router,
                        int othernode, int otherlink, int otherrouter)
{
        NETLINK *l;

        if (nNetLink >= MAX_NETLINK)
        {
                printf ("-E-EMU414: More than %d links in the network map.\n", MAX_NETLINK);
                return (-1);
        }
        if (verbose)
                printf ("-I-EMU414: Connect %d-%d => %d-%d\n", node, link, othernode, otherlink);
        l = &NetLinks[nNetLink++];
        l->node = node;
        l->link = link;
        l->router = router;
        l->othernode = othernode;
        l->otherlink = otherlink;
        l->otherrouter = otherrouter;
        return 0;
}

/* A cable may be listed twice, but a link belongs to one cable: 1 if this
 * cable is known already, -1 if the link is on another one. */
static int link_in_use (int node, int link, int othernode, int otherlink, int nline)
{
        int on, ol;

        if (node < 0 || connectedNetLink (node, link, &on, &ol) < 0)
                return 0;
        if (on == othernode && ol == otherlink)
                return 1;
        printf ("-E-EMU414: Node %d Link%d at line %d is already connected to node %d Link%d.\n",
                node, link, nline, on, ol);
        return (-1);
}

#define MAX_NETLINE 512

int readNetConfig (FILE *fin)
{
        char buf[MAX_NETLINE];
        int i, nline;
        char *ptr;
        int node, link, thenode, thelink, therouter, known, knownback;
        char url[MAX_NETLINE];

        network = 0xC0A80000; /* 192.168.0.0 */
        address = 0;
        port = 1984;
        scale = 4;

        max_nodeid = -1;

        nline = 1;
        ptr = fgets (buf, sizeof(buf), fin);
        while (ptr)
        {
                /* fprintf (stderr, "line = [%s]\n", buf); */
                ptr = skip (ptr);
                if (0 == *ptr)
                        goto NextLine;

                if (0 == strncmp (ptr, "--", 2))
                        goto NextLine;
                else if (0 == strncmp (ptr, "Connect ", 8))
                {
                        /* Connect HOST to processor <number> link <number> */
                        /* Connect processor <number> link <number>  to processor <number> link <number> */
                        ptr = read_link (ptr + 8, &thenode, &thelink);
                        therouter = link_router;
                        if (NODE_INVALID == thenode)
                        {
                                printf ("-E-EMU414: Cannot read link at line %d.\n", nline);
                                return (-1);
                        }
                        if (thenode >= 0)
                        {
                                ptr = skip (ptr);
                                if (strncmp (ptr, "to", 2))
                                {
                                        printf ("-E-EMU414: Missing 'to' at line %d.\n", nline);
                                        return (-1);
                                }
                                ptr = read_link (ptr + 2, &node, &link);
                                if (NODE_INVALID == node)
                                {
                                        printf ("-E-EMU414: Cannot read 'to' link at line %d.\n", nline);
                                        return (-1);
                                }
                                known = link_in_use (thenode, thelink, node, link, nline);
                                knownback = link_in_use (node, link, thenode, thelink, nline);
                                if (known < 0 || knownback < 0)
                                        return (-1);
                                if (!known &&
                                    add_netlink (thenode, thelink, therouter, node, link, link_router) < 0)
                                        return (-1);
                                if (!knownback &&
                                    add_netlink (node, link, link_router, thenode, thelink, therouter) < 0)
                                        return (-1);
                        }
                }
                else if (0 == strncmp (ptr, "network", 7))
                {
                        /* network <number> */
                        ptr = read_number (ptr + 7, &network);
                        if (-1 == network)
                        {
                                printf ("-E-EMU414: Invalid network address at line %d.\n", nline);
                                return (-1);
                        }
                        address = 0;
                        dump_netparams ();
                }
                else if (0 == strncmp (ptr, "address", 7))
                {
                        /* address <number> */
                        ptr = read_number (ptr + 7, &address);
                        if (-1 == address)
                        {
                                printf ("-E-EMU414: Invalid address at line %d.\n", nline);
                                return (-1);
                        }
                        network = 0;
                        dump_netparams ();
                }
                else if (0 == strncmp (ptr, "port", 4))
                {
                        /* port <number> */
                        ptr = read_number (ptr + 4, &port);
                        if (-1 == port)
                        {
                                printf ("-E-EMU414: Invalid port at line %d.\n", nline);
                                return (-1);
                        }
                        dump_netparams ();
                }
                else if (0 == strncmp (ptr, "scale", 5))
                {
                        /* scale <number> */
                        ptr = read_number (ptr + 5, &scale);
                        if (-1 == scale)
                        {
                                printf ("-E-EMU414: Invalid scale at line %d.\n", nline);
                                return (-1);
                        }
                        dump_netparams ();
                }
                else if (0 == strncmp (ptr, "link", 4))
                {
                        /* "link" <link> string */
                        /* "link" "IPC"|"SHM", default "IPC" */
                        ptr = read_link (ptr + 4, &node, &link);
                        if (NODE_SHM == node)
                        {
                                shlinks = 1;
                                if (verbose)
                                        printf ("-I-EMU414: Default link protocol is SHM.\n");
                        }
                        else if (NODE_IPC == node)
                        {
                                shlinks = 0;
                                if (verbose)
                                        printf ("-I-EMU414: Default link protocol is IPC.\n");
                        }
                        else if (node < 0)
                        {
                                printf ("-E-EMU414: Invalid link at line %d.\n", nline);
                                return (-1);
                        }
                        else
                        {
                                ptr = read_string (ptr, url);
                                if (0 == url[0])
                                {
                                        printf ("-E-EMU414: Missing URL at line %d.\n", nline);
                                        return (-1);
                                }
                                add_url (node, link, url);
                        }
                }
                else if (0 == strncmp (ptr, "node", 4))
                {
                        int j, p;
                        char tmpstr[MAX_NETLINE];
                        char *colon1, *colon2;

                        p = port;

                        /* "node" from[-to] [proto://address[:port]] */
                        ptr = read_number (ptr + 4, &thenode);
                        if (-1 == thenode)
                        {
                                printf ("-E-EMU414: Invalid node at line %d.\n", nline);
                                return (-1);
                        }
                        node = thenode;
                        if (0 == strncmp (ptr, "-", 1))
                        {
                                ptr = read_number (ptr + 1, &node);
                                if (-1 == node)
                                {
                                        printf ("-E-EMU414: Invalid node at line %d.\n", nline);
                                        return (-1);
                                }
                        }
                        tmpstr[0] = '\0';
                        ptr = skip (ptr);
                        if (0 == *ptr)
                                goto ProcessNode;
                        if (0 == strncmp (ptr, "--", 2))
                        {
                                goto ProcessNode;
                        }
                        ptr = read_string (ptr, tmpstr);
                        if (*tmpstr)
                        {
                                /* proto://address[:port] */
                                colon1 = strchr (tmpstr, ':');
                                colon2 = strrchr (tmpstr, ':');
                                if (NULL == colon1)
                                {
                                        printf ("-E-EMU414: Invalid URL at line %d.\n", nline);
                                        return (-1);
                                }
                                if (colon1 == colon2) /* proto://address */
                                        p = port;
                                else
                                {
                                        /* parse port, make proto://address */
                                        p = atoi (colon2 + 1);
                                        *colon2 = '\0';
                                }
                        }
ProcessNode:
                        /* define links */
                        for (i = thenode; i <= node; i++)
                                for (j = 0; j < 4; j++)
                                {
                                        if (*tmpstr)
                                        {
                                                int temp = p + scale * (i - thenode) + j; /* relative */
                                                if (p == port)
                                                        temp = p + scale * i + j; /* absolute */
                                                sprintf (url, "%s:%d", tmpstr, temp);
                                        }
                                        else
                                                strcpy (url, tcpLinkURL (i, j));
                                        add_url (i, j, url);
                                }
                }
                else if (0 == strncmp (ptr, "router", 6) && isspace (ptr[6]))
                {
                        if (read_router (ptr + 6, nline) < 0)
                                return (-1);
                }
                else if (isdigit(*ptr))
                {
                        /* number <link> <link> <link> <link>
                         * link := "host" | "-" | number "-" number
                         */
                        ptr = read_number (ptr, &thenode);
                        if (-1 == node)
                        {
                                printf ("-E-EMU414: Cannot read node id at line %d.\n", nline);
                                return (-1);
                        }
                        for (i = 0; i < 4; i++)
                        {
                                ptr = skip (ptr);
                                if (0 == *ptr)
                                        break;
                                if (0 == strncmp (ptr, "--", 2))
                                        break;
                                ptr = read_link (ptr, &node, &link);
                                if (NODE_INVALID == node)
                                {
                                        printf ("-E-EMU414: Cannot read link at line %d.\n", nline);
                                        return (-1);
                                }
                                if (node >= 0 && /* skip HOST and NOT CONNECTED links */
                                    add_netlink (thenode, i, 0, node, link, link_router) < 0)
                                        return (-1);
                        }
                }
NextLine:
                ptr = fgets (buf, sizeof(buf), fin);
                nline++;
        }
        return check_routers ();
}

int connectedNetLink(int node, int link, int *othernode, int *otherlink)
{
        int i;

        for (i = 0; i < nNetLink; i++)
        {
                if ((node == NetLinks[i].node) && (link == NetLinks[i].link))
                {
                        *othernode = NetLinks[i].othernode;
                        *otherlink = NetLinks[i].otherlink;
                        return 0;
                }
        }
        return -1;
}

/* connectedNetLink, or failing that a connection given only from the other
 * end (a numeric line describes just that node's own links). */
int linkedNetLink (int node, int link, int *othernode, int *otherlink)
{
        int i;

        if (0 == connectedNetLink (node, link, othernode, otherlink))
                return 0;
        for (i = 0; i < nNetLink; i++)
        {
                if ((node == NetLinks[i].othernode) && (link == NetLinks[i].otherlink))
                {
                        *othernode = NetLinks[i].node;
                        *otherlink = NetLinks[i].link;
                        return 0;
                }
        }
        return -1;
}

char* netLinkURL (int node, int link)
{
        static char buf[128];
        int i;

        for (i = 0; i < nNetURL; i++)
        {
                if ((node == NetURLs[i].node) && (link == NetURLs[i].link))
                        return NetURLs[i].url;
        }
        sprintf (buf, "ipc:///tmp/node%d-%d.lnk", node, link);
        return buf;
}

int sharedLinks (void)
{
        return shlinks;
}

int maxNodeID (void)
{
        int i, ret;

        if (max_nodeid >= 0)
                return max_nodeid;

        ret = -1;
        for (i = 0; i < nNetLink; i++)
        {
                if (NetLinks[i].node > ret)
                        ret = NetLinks[i].node;
                if (NetLinks[i].othernode > ret)
                        ret = NetLinks[i].othernode;
        }

        if (-1 == ret) ret = 0;
        max_nodeid = ret;
        return max_nodeid;
}

#if defined(NETCFG_TEST)
int verbose = 1;
int shlinks = 0;
int main(int argc, char*argv[])
{
        FILE *fin;

        if (2 != argc)
        {
                fprintf (stderr, "usage: netcfg <file>\n");
                exit (1);
        }
        if ((fin = fopen (argv[1], "r")) == NULL)
        {
                fprintf (stderr, "cannot open %s\n", argv[1]);
                exit (1);
        }
        readNetConfig (fin);
        fclose (fin);

        return 0;
}
#endif
