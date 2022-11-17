---
title: "DatKom Notes"
read_time: false
excerpt: "For learning DatKom"
header:
  teaser: /assets/images/C_logo.png
  overlay_image: /assets/images/C_logo.png
  overlay_filter: 0.5 
toc: true
toc_label: "Contents"
toc_sticky: true
categories:
  - Notes
tags:
  - datkom
  - notes

---

# Concepts

- Communication services, layer models, protocols
- Physical basics of transmission
- Error handling and medium access
- Internet Protocol (IP) and Routing: Connecting remote hosts
- Transmission Control Protocol (TCP): Connecting applications
- Security: Cryptographic primitives, IPsec, SSL/TLS

From [source](https://www.campus.rwth-aachen.de/rwth/all/abstractModule.asp?gguid=0x2F22AE7D7BE14B4B8AEC4D8B904EC25A&tguid=0x52DF76AB4F0BB84AB1CAEF0A89F08202):
- Client/Server- und Peer-to-Peer-Systeme
- OSI-Referenzmodell und TCP/IP-Referenzmodell
- Übertragungsmedien und Signaldarstellung
- Fehlerbehandlung, Flusssteuerung und Medienzugriff
- Lokale Netze, speziell Ethernet
- Netzkomponenten und Firewalls
- Internet-Protokolle: IP, Routing, TCP/UDP
- Sicherheitsmanagement und Datenschutz, Sicherheitsprobleme und Angriffe im Internet
- Grundlagen der Kryptographie und sichere Internet-Protokolle

# What every CS major should know

([source](https://matt.might.net/articles/what-cs-majors-should-know/))

## Networking

Given the ubiquity of networks, computer scientists should have a firm understanding of the network stack and routing protocols within a network.

The mechanics of building an efficient, reliable transmission protocol (like TCP) on top of an unreliable transmission protocol (like IP) should not be magic to a computer scientist. It should be core knowledge.

Computer scientists must understand the trade-offs involved in protocol design--for example, when to choose TCP and when to choose UDP. (Programmers need to understand the larger social implications for congestion should they use UDP at large scales as well.)

### Specific recommendations

Given the frequency with which the modern programmer encounters network programming, it's helpful to know the protocols for existing standards, such as:

- 802.3 and 802.11;
- IPv4 and IPv6; and
- DNS, SMTP and HTTP.

Computer scientists should understand exponential back off in packet collision resolution and the additive-increase multiplicative-decrease mechanism involved in congestion control.

Every computer scientist should implement the following:

- an HTTP client and daemon;
- a DNS resolver and server; and
- a command-line SMTP mailer.

No student should ever pass an intro neworking class without sniffing their instructor's Google query off wireshark.

It's probably going too far to require all students to implement a reliable transmission protocol from scratch atop IP, but I can say that it was a personally transformative experience for me as a student.

### Recommended reading

- Unix Network Programming by Stevens, Fenner and Rudoff.

## Security

The sad truth of security is that the majority of security vulnerabilities come from sloppy programming. The sadder truth is that many schools do a poor job of training programmers to secure their code.

Computer scientists must be aware of the means by which a program can be compromised.

They need to develop a sense of defensive programming--a mind for thinking about how their own code might be attacked.

Security is the kind of training that is best distributed throughout the entire curriculum: each discipline should warn students of its native vulnerabilities.

### Specific recommendations

At a minimum, every computer scientist needs to understand:

- social engineering;
- buffer overflows;
- integer overflow;
- code injection vulnerabilities;
- race conditions; and
- privilege confusion.

A few readers have pointed out that computer scientists also need to be aware of basic IT security measures, such how to choose legitimately good passwords and how to properly configure a firewall with iptables.

### Recommended reading

- Metasploit: The Penetration Tester's Guide by Kennedy, O'Gorman, Kearns and Aharoni.
- Security Engineering by Anderson.

## Cryptography

Cryptography is what makes much of our digital lives possible.

Computer scientists should understand and be able to implement the following concepts, as well as the common pitfalls in doing so:

- symmetric-key cryptosystems;
- public-key cryptosystems;
- secure hash functions;
- challenge-response authentication;
- digital signature algorithms; and
- threshold cryptosystems.

Since it's a common fault in implementations of cryptosystems, every computer scientist should know how to acquire a sufficiently random number for the task at hand.

At the very least, as nearly every data breach has shown, computer scientists need to know how to salt and hash passwords for storage.

### Specific recommendations

Every computer scientist should have the pleasure of breaking ciphertext using pre-modern cryptosystems with hand-rolled statistical tools.

RSA is easy enough to implement that everyone should do it.

Every student should create their own digital certificate and set up https in apache. (It's surprisingly arduous to do this.)

Student should also write a console web client that connects over SSL.

As strictly practical matters, computer scientists should know how to use GPG; how to use public-key authentication for ssh; and how to encrypt a directory or a hard disk.

### Recommended reading

- Cryptography Engineering by Ferguson, Schneier and Kohno.

# Linux commands

Base64 decode:
```bash
echo -n 'UGFzc3dvcmQ6' | base64 -d
```

Base64 encode:
```bash
echo -n 'Password:' | base64
```

Hex to decimal number:
```bash
printf "%d\n" 0xFF
```

# Wireshark GUI meaning

## The "Packet List" Pane

- the lines in the **"No." column** connecting the selected packet with other packets ([see Table 3.16. Related packet symbols](https://www.wireshark.org/docs/wsug_html_chunked/ChUsePacketListPaneSection.html))
    - DNS packets that use the **same port numbers**. Wireshark treats them as belonging to the **same conversation** and draws a line connecting them.

# Multiplexing

## Circuit Switching (TDM or FDM)

- In **circuit switching** network resources (bandwidth) are divided into pieces (using either **TD multiplexing (TDM)** or **FD multiplexing (FDM)**)
- bit delay is constant during a connection

## Packet Switching (statistical multiplexing)

- transfers the data to a network in form of **packets** (using **statistical multiplexing**)
- unlike circuit switching, packet switching requires no pre-setup (saving time) or reservation of resources (saving resources)
- from [geeksforgeeks](https://www.geeksforgeeks.org/packet-switching-and-delays-in-computer-network/?ref=lbp)
    - Packet Switching uses **Store and Forward** technique while switching the packets; while forwarding the packet each hop first stores that packet then forward. 
        - This technique is very **beneficial because** packets may get discarded at any hop due to some reason. 
        - **More than one path is possible** between a pair of sources and destinations. Each packet contains Source and destination address using which **they independently travel** through the network. 
            - In other words, packets belonging to the same file may or may not travel through the same path. If there is **congestion** at some path, packets are allowed to choose different paths possible over an existing network. 

# The Internet

- An **Internet service provider (ISP)** is an organization that provides services for accessing, using, or participating in the Internet.
- **Internet exchange points** (**IXes** or **IXPs**) are common grounds of IP networking, allowing participant Internet service providers (ISPs) to exchange data destined for their respective networks

## Tiers of networks

- There is no authority that defines tiers of networks participating in the Internet
- A **Tier 1 network** is a network that can reach every other network on the Internet without purchasing IP transit or paying for peering
- A **Tier 2 network** is an Internet service provider which engages in the practice of peering with other networks, but which also purchases IP transit to reach some portion of the Internet.
    - the most common Internet service providers, as it is much easier to purchase transit from a Tier 1 network than to peer with them and attempt to become a Tier 1 carrier
- The term **Tier 3 network** is sometimes also used to describe networks who solely purchase IP transit from other networks to reach the Internet

# Delay and Loss

- delay
    - transmission delay
        - takes some time to put the bits on the wire (or whatever the medium is)
    - queueing delay
        - if there is a line, i.e. some packets are waiting to be sent, there will be a delay
- loss
    - arriving packets will be dropped by the router, if the queue is full, i.e. if there are no free buffers

# Application layer (Brief)

- **ip address**: identifies a specific host
- **port**: identifies a specific process
    - e.g. **http servers**/web servers run on **port 80**
        - when **browsers** want http files, they will send a message to the ip address of the server and the port number associated with http, i.e. port 80

# Transport layer (Brief)

## Transport protocols: TCP/UDP

- TCP
    - **connection-oriented**: sets up a connection between client and server processes (thus, there is some overhead to set this up initially)
    - **reliable**: makes sure there is no packet loss (requires some extra information to be sent, i.e. reliability requires some extra bandwidth)
    - **flow control**: slows down the sender because the **receiver** is overloaded
    - **congestion control**: slows down the sender because the **network** (in between the sender and receiver) is overloaded
- UDP
    - **connection-less**: no setting up, thus no overhead
    - **unreliable**: no guarantees that a sent packet will arrive or will be uncorrupted or will be unduplicated
    - **no flow control**
    - **no congestion control**
- **note**: UDP is faster and gives you more control over the trade-offs you want to make than TCP. If you need reliability, flow control or congestion control you could implement it yourself at the application layer. For some apps UDP makes more sense, e.g. internet telephony, streaming video.
    - from wiki: "Voice and video traffic is generally transmitted using UDP"

# Application Layer

## Web and HTTP

TODO

## Email

### SMTP

Using **app passwords** in Gmail: "When you use 2-Step Verification, some less secure apps or devices may be blocked from accessing your Google Account. App Passwords are a way to let the blocked app or device access your Google Account."

Mostly from [stackoverflow](https://stackoverflow.com/a/13772865/12282296):
```bash
# must use -ign_eof flag here, otherwise the "R" in RCPT TO line will cause "RENEGOTIATING"
# (see https://stackoverflow.com/questions/59956241/connection-with-google-mail-using-openssl-s-client-command)
$ openssl s_client -connect smtp.gmail.com:465 -ign_eof

HELO smtp

auth login

# get username in base64 encoding 
# (sidenote: in order to *DE*code simply use "base64 -d" instead of "base64" at the end of this command)
echo -n 'phrth2@gmail.com' | base64

# generate app password (see https://support.google.com/accounts/answer/185833?hl=en)
echo -n 'app_password' | base64

EHLO smtp
MAIL FROM: <phrth@gmx.de>
RCPT TO: <phrth2@gmail.com>
DATA

=== enter e-mail content ===

ctrl-v-enter enter
. ctrl-v-enter
```

## DNS

- A Distributed, Hierarchical Database
- hierarchy:
    - **root DNS server** (returns IP address of TLD server, e.g. for TLD `.com`)
    - **TLD server** (returns IP address of authoritative server, e.g. authoritative for `amazon.com`)
        - **Note**: The TLD server does not always know the authoritative DNS server for the hostname! It may know only of an **intermediate DNS server**, which in turn knows the authoritative DNS server for the hostname.
            - e.g. an intermediate university DNS server and authoritative departmental DNS servers
    - (intermediate DNS server)
    - **authoritative DNS server** (returns the IP address for the hostname, e.g. for the hostname `www.amazon.com`)
        - most universities and large companies implement and maintain their own authoritative server
- **local DNS server** (aka **default name server**): 
    - does not belong to the hierarchy
    - each ISP has a local DNS server
    - when a host connects to an ISP, the ISP provides the host with the IP addresses of one or more of its local DNS servers (via **DHCP**)
    - typically close to the host (i.e. not more than a few routers away from the host)

### Recursive vs Iterative DNS query

- **recursive DNS query**: obtain the mapping on the querying DNS server's behalf
- **iterative DNS query**: replies are directly returned to the querying DNS server
- in theory, any DNS query can be **iterative** or **recursive**
- in practice, DNS queries typically follow the pattern in Fig. 2.19
    - i.e. the query from the requesting host to the local DNS server is recursive, and the remaining queries are iterative

### DNS Cache

- In a query chain, when a DNS server receives a DNS reply (containing, for example, a mapping from a hostname to an IP address), it can **cache** the mapping in its local memory.
- **Timeout**: Because hosts and mappings between hostnames and IP addresses are by no means permanent, DNS servers **discard** cached information after a period of time (often set to two days)
- A local DNS server can also cache the **TLD server**s' IP addresses
    - In fact, because of caching, **root servers** are bypassed for all but a very small fraction of DNS queries

#### DNS Cache in Ubuntu, Time to Live (TTL)

From [linuxhint.com](https://linuxhint.com/flush_dns_cache_ubuntu/) and [techrepublic.com](https://www.techrepublic.com/article/how-to-view-dns-cache-entries-with-the-new-systemd-resolved-resolver/):

Ubuntu caches DNS queries by default using **systemd-resolved**. 
- `systemd-resolved` is a **systemd service** that provides network name resolution *to local applications* via a **local DNS stub listener** on `127.0.0.53`. (from [wiki.archlinux](https://wiki.archlinux.org/title/systemd-resolved))
    - It implements a **caching** and validating DNS/DNSSEC stub resolver (from [freedesktop.org](https://www.freedesktop.org/software/systemd/man/systemd-resolved.service.html))
- Prior to systemd, there was almost no OS-level DNS caching, see [stackexchange](https://unix.stackexchange.com/a/211396)
    - old Ubuntu versions used `dnsmasq` (A lightweight DHCP and caching DNS server) and `nscd` (name service cache daemon) instead of `systemd-resolved`

```bash
# check how many DNS entries are cached
sudo systemd-resolve --statistics

# flush the DNS cache
sudo systemd-resolve --flush-caches

# alternatively, restart the systemd-resolved service to flush the DNS caches
sudo systemctl restart systemd-resolved

# view the DNS cache contents
sudo killall -USR1 systemd-resolved
sudo journalctl -u systemd-resolved > ~/dns-cache.txt
```

First check, if DNS caching is enabled:

If DNS caching is **enabled** the DNS server used to resolve the domain name (e.g. when using `nslookup domain_name`) is a [loopback IP address](#loopback-address), e.g. 127.0.0.53. If you have it **disabled**, then the DNS server should be anything other than 127.0.0.X.
    - **Why?**: This is because the loopback ip address `127.0.0.X` indicates that the service `systemd-resolved` (which caches DNS records) is running.

Then, in order to get **TTL values**:

```bash
nslookup -debug www.cyberciti.biz
nslookup -debug -type=NS google.com

dig +ttlunits A www.cyberciti.biz
dig +ttlunits NS google.com

# show the TTL only
dig +nocmd +noall +answer +ttlid A www.cyberciti.biz
```

### DNS Resource Records (RRs)

- a four-touple `(Name, Value, Type, TTL)`, where `Name` and `Value` depend on the `Type`
- If a DNS server is **authoritative** for a particular hostname, then the DNS server will contain a **Type A record** for the hostname. 
    - (Even if the DNS server is not authoritative, it may contain a Type A record in its **cache**.) 
- If a server is **not authoritative** for a hostname, then the server will contain a **Type NS record** for the domain that includes the hostname
    - it will also contain a **Type A record** that provides the IP address of the DNS server in the Value field of the NS record.

#### nslookup, dig

**Caution**: `dig` and `nslookup` sometimes display different information. `dig` uses the OS resolver libraries. `nslookup` uses its own internal ones. ISC recommends to stop using `nslookup` (see [stackexchange](https://unix.stackexchange.com/a/93809))!

**nslookup**:
General syntax: `nslookup –option1 –option2 host-to-find dns-server`
Options:
- `nslookup -debug`: show additional information, e.g. TTL

**dig**:
General syntax: `dig [options] TYPE domain auth-name-server-here`
Options:
- `dig +short`: short form answer
- `dig +ttlunits`: human-readable time units
- `dig +nocmd +noall +answer +ttlid`: show the TTL only

**Example 1**: Send me the IP address for the **host** www.mit.edu:
```bash
nslookup -type=A www.mit.edu
dig A www.mit.edu
```

**Example 2**: Send me the host names of the authoritative DNS servers for the **domain** mit.edu:
```bash
# note: mit.edu is a domain, so no "www."!
nslookup -type=NS mit.edu
dig NS mit.edu
```

**Example 3**: Query sent to the DNS server `dns2.p08.nsone.net` rather than to the default DNS server:
- possible errors, when using `nslookup`:
    - `** server can't find some_site.com: NXDOMAIN`: the `host-to-find` is wrong
    - `** server can't find some_site.com: REFUSED`: the `dns-server` is wrong, more precisely:
        - [nslookup from another name server](https://superuser.com/a/1624001)
        - ["Refused" doesn't mean that the connection to the DNS server is refused. It means that the DNS server refuses to provide whatever data you asked for, or to do whatever action you asked it to do](https://stackoverflow.com/a/1697403/12282296)

```bash
nslookup www.google.com dns2.p08.nsone.net

# note: do not forget the @ symbol here!
dig www.google.com @dns2.p08.nsone.net
```

### ip, ifconfig, ipconfig, nmcli

- `ipconfig` is a Windows command, it displays slightly different information than `ifconfig` and `ip`
- `ifconfig` is deprecated, use `ip`
- `ip a`, `ip address`, `ip address show`
- `ip r` default gateway/router address
- `nmcli dev show wlo1` show gateway and DNS servers for device `wlo1`
- `nmcli dev status` get device names and their status types
- `systemd-resolve --status | grep Current`: currently used DNS server IP address ([source](https://linuxconfig.org/how-to-find-my-ip-address-on-ubuntu-20-04-focal-fossa-linux))

### Loopback address, Localhost

From [techopedia.com](https://www.techopedia.com/definition/2440/loopback-address-ip-address):

A **loopback address** has been built into the IP domain system in order to allow for a device to send and receive its own data packets.

Loopback addresses can be useful in various kinds of analysis like testing and debugging, or in allowing routers to communicate in specific ways.

A simple way of describing how using a loopback address works is that a data packet will get sent through a network and routed back to the same device where it originated.

In IPv4, **127.0.0.1** is the most commonly used loopback address, however, this can range be extended to **127.255.255.255**.

From [tutorialspoint.com](https://www.tutorialspoint.com/ipv4/ipv4_reserved_addresses.htm):

The IP address range 127.0.0.0 – 127.255.255.255 is reserved for loopback, i.e. a Host's self-address, also known as **localhost address**. This loopback IP address is managed entirely by and within the operating system. Loopback addresses, enable the Server and Client processes on a single system to communicate with each other. When a process creates a packet with destination address as loopback address, the operating system loops it back to itself without having any interference of NIC ([wiki "NIC"](https://en.wikipedia.org/wiki/Network_interface_controller): **network interface controller** (**NIC**, also known as a **network interface card**, **network adapter**, **LAN adapter** or **physical network interface**).

Data sent on loopback is forwarded by the operating system to a **virtual network interface** within operating system. This address is mostly used for testing purposes like client-server architecture on a single machine. Other than that, if a host machine can successfully ping 127.0.0.1 or any IP from loopback range, implies that the TCP/IP software stack on the machine is successfully loaded and working.

Why is such a **large IPv4 range assigned to localhost**? (from [stackexchange](https://networkengineering.stackexchange.com/questions/2840/why-is-such-a-large-ipv4-range-assigned-to-localhost))

At the time (1986), the internet was completely classful and nobody really gave much thought to allocating this much space to the loopback address. Thus, the loopback got an entire Class A network.

## P2P File Distribution

TODO

# Transport Layer

The **Transport Layer** is about providing **logical communication** between **processes**.
    - in other words **TCP and UDP** are **end to end** protocols (and not "point to point"!)

The transport layer of the sender **disassembles** (multiplexing) the messages passed from the application layer into chunks and the receiver **reassembles** (demultiplexing) those messages.

## End-to-End Principle

see 
- [explanation on youtube](https://www.youtube.com/watch?v=3Iy4EQpGnpo)
- [devopedia.org](https://devopedia.org/end-to-end-principle)

Basic design principle of the internet: "keep the core simple".

There are many different phrasings:
- "Communication protocol operations should be defined to occur at the **endpoints** of a communication system."
- Saltzer 1984: "The principle, called the **end-to-end argument**, suggests that functions placed at low levels of a system may be **redundant** or of **little value** when compared with the cost of providing them at that low level."

Examples:
- TCP checksum vs. application checksum
- "smart" TCP vs "dumb" IP
