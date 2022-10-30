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

From [[source](https://www.campus.rwth-aachen.de/rwth/all/abstractModule.asp?gguid=0x2F22AE7D7BE14B4B8AEC4D8B904EC25A&tguid=0x52DF76AB4F0BB84AB1CAEF0A89F08202)]:
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

[[source](https://matt.might.net/articles/what-cs-majors-should-know/)]

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

# Application layer

- **ip address**: identifies a specific host
- **port**: identifies a specific process
    - e.g. **http servers**/web servers run on **port 80**
        - when **browsers** want http files, they will send a message to the ip address of the server and the port number associated with http, i.e. port 80

# Transport layer

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

# Email

## SMTP

- using **app passwords** in Gmail: "When you use 2-Step Verification, some less secure apps or devices may be blocked from accessing your Google Account. App Passwords are a way to let the blocked app or device access your Google Account."
- Mostly from [stackoverflow](https://stackoverflow.com/a/13772865/12282296):
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

# DNS

- **local DNS server** (aka **default name server**): 
    - does not belong to the hierarchy
    - each ISP has a local DNS server
    - when a host connects to an ISP, the ISP provides the host with the IP addresses of one or more of its local DNS servers (via **DHCP**)
    - typically close to the host (i.e. not more than a few routers away from the host)
- in theory, any DNS query can be **iterative** or **recursive**
- in practice, DNS queries typically follow the pattern in Fig. 2.19
    - i.e. the query from the requesting host to the local DNS server is recursive, and the remaining queries are iterative

## DNS Cache

- In a query chain, when a DNS server receives a DNS reply (containing, for example, a mapping from a hostname to an IP address), it can **cache** the mapping in its local memory.
- **Timeout**: Because hosts and mappings between hostnames and IP addresses are by no means permanent, DNS servers **discard** cached information after a period of time (often set to two days)
- A local DNS server can also cache the **TLD server**s' IP addresses
    - In fact, because of caching, **root servers** are bypassed for all but a very small fraction of DNS queries
