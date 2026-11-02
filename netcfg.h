#ifndef _NETCFG_H
#define _NETCFG_H

int readNetConfig (FILE *fin);
int connectedNetLink (int node, int link, int *othernode, int *otherlink);
int linkedNetLink (int node, int link, int *othernode, int *otherlink);
char *netLinkURL (int node, int link);
int sharedLinks (void);
int maxNodeID (void);

/* IMS C104 packet routing switches in the network map (HRM part 1 ch.7,
 * part 3 ch.2).  A static description of what the control link would
 * program on real hardware; the switch itself lives in t4_fabric.c. */
#define NET_ROUTER_LINKS        32

typedef struct net_interval {
        unsigned lo, hi;        /* lo <= header < hi */
        int first, last;        /* output link, or a group of links */
        unsigned on;            /* input links whose selector holds it */
} net_interval_t;

typedef struct net_router {
        int id, nlinks;
        unsigned hdr2;          /* input links reading 2-byte headers */
        unsigned del;           /* output links deleting the header */
        int nint;
        net_interval_t *ints;
} net_router_t;

int netRouterCount (void);
net_router_t *netRouter (int i);
net_router_t *netRouterById (int id);

#endif
