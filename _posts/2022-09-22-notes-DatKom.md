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

