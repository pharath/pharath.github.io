---
title: "Web3 and Distributed Ledger Technology - SSI"
read_time: false
excerpt: "For studying \"Web3 and Distributed Ledger Technology\"; content mostly from \"RWTH lecture Web3 and Distributed Ledger Technology\" by Wolfgang Prinz."
toc: true
toc_label: "Contents"
toc_sticky: true
toc_h_max: 3
author_profile: false
categories:
  - Notes
tags:
  - web3
  - notes
  - lecture

---

# SSI

## slide: "Basics - Digital Identity"

### ChatGPT: "meaning"

The slide explains that **digital identity** is the collection of data used to **recognize, authenticate, and describe a person or entity online** when using internet services.

#### Meaning of each point

##### 1. Login (username, password, PIN)

These are **credentials** used to prove that you are the authorized user.

Examples:

* Username + password for email
* PIN for banking app
* Fingerprint or face ID (sometimes replaces PIN/password)

➡️ Purpose: **Authentication** (proving who you are)

---

##### 2. Profile data (name, email, address, date of birth, etc.)

This is personal information linked to your account.

Examples:

* Full name
* Email address
* Shipping address
* Birthday

➡️ Purpose: **Identification and personalization**

Used by companies to know who you are, contact you, or tailor services.

---

##### 3. Device or system data (IP address, device ID, certificates)

Technical data from the device or network you use.

Examples:

* IP address (internet connection identifier)
* Device ID (phone/computer identifier)
* Browser fingerprint
* Security certificate

➡️ Purpose:

* Security (fraud detection, suspicious login alerts)
* Access control
* Device recognition

---

##### 4. Usage behaviour data (tracking)

Data about how you use a service.

Examples:

* Pages visited
* Time spent on app
* Clicks
* Purchases
* Search history

➡️ Purpose:

* Analytics
* Recommendations
* Advertising
* Improving services

---

#### Overall Meaning

Your **digital identity** is not just your username or name. It is a combination of:

1. **What you know** (password, PIN)
2. **Who you are** (profile data)
3. **What you use** (device/system data)
4. **How you behave** (usage data)

Together, these allow companies, platforms, and systems to identify or profile you online.

---

#### Why it matters in Web3 / SSI

Traditional Web2 companies store and control this identity data centrally.
In **Web3 / Self-Sovereign Identity (SSI)**, the goal is for users to control their own identity data using tools like **DIDs** and **Verifiable Credentials**.

---

#### One-line summary

**Digital identity = all data that represents and distinguishes you in the digital world.**

## slide: "Stages of Identity Management (IDM) - Digital Identity"

### ChatGPT: "meaning"

This table explains the **evolution of Identity Management (IDM)** for **digital identity** — from systems controlled by central institutions to models where the user controls identity themselves.

#### What is Identity Management (IDM)?

Identity Management means:

* creating digital identities
* logging in users
* verifying who someone is
* managing access rights
* storing identity data

---

#### 1. Centralized Identity Management

**Meaning:**
A single central authority manages identities or core internet identifiers.

Examples:

* ICANN → manages domain name system coordination
* IANA → coordinates IP address allocations
* Certification Authorities → issue SSL/TLS certificates

##### Characteristics:

* Strong dependence on one trusted institution
* Single point of failure
* Users have little or no control

##### Example:

If a certificate authority is compromised, many websites can be affected.

---

#### 2. Service-Based Identity Management

**Meaning:**
Each website or service creates and manages its own user accounts.

Examples:

* Online shops
* Email providers
* Forums
* Streaming platforms

##### Characteristics:

* Separate login for every service
* User manages many passwords

##### Problems:

* Password reuse
* Forgotten passwords
* Account chaos

##### Example:

Different passwords for Amazon, Netflix, university portal, bank, etc.

---

#### 3. Identity Provider (IdP) Model

**Meaning:**
A third-party provider handles authentication for many services.

Examples:

* Google Sign-In
* Facebook Login
* Apple Sign in with Apple

##### Characteristics:

* Convenient single sign-on (SSO)
* One account for many websites

##### Problems:

* Large concentration of user data
* Tracking across services
* Privacy loss

##### Example:

Using Google login on many websites lets Google know where you log in.

---

#### 4. Self-Sovereign Identity (SSI)

**Meaning:**
Users own and control their identities and credentials directly.

Examples:

* Digital credentials in wallet apps
* Decentralized Identifiers (DIDs)
* Verifiable Credentials

##### Characteristics:

* No central provider required
* User stores credentials in wallet
* Selective disclosure possible

##### Advantages:

* Better privacy
* More security
* User control

##### Challenges:

* Users must protect wallet/keys
* Adoption still growing

---

#### Overall Meaning of the Table

It shows a shift:

**Centralized control → Many separate accounts → Big login providers → User-controlled identity**

So digital identity systems are moving from **institution-owned identity** toward **user-owned identity**.

---

#### One-Line Summary

**SSI is the newest IDM stage where the user, not a platform, controls digital identity and credentials.**

## slide: "Principles - Self-Sovereign Identity"

### ChatGPT: "meaning"

This table explains the **core principles of Self-Sovereign Identity (SSI)** — a model where people **own, control, and manage their digital identity themselves**, instead of relying on governments, platforms, or big tech providers.

Each principle describes what a fair digital identity system should provide.

---

#### Meaning of Each Principle

##### 1. Existence

**Users must have an independent existence.**

Meaning:
Your identity should exist independently of any company or platform.

###### Benefit:

* Not dependent on central control
* Your identity does not disappear if a company shuts down your account

**Example:** Your identity should not only exist because Google or Facebook says so.

---

##### 2. Control

**Users must control their own identity.**

Meaning:
You decide who can use your identity data.

###### Benefit:

* Prevents unauthorized tracking
* Reduces identity theft

**Example:** You choose when to share your email or address.

---

##### 3. Access

**Users must have access to their own data.**

Meaning:
You should always be able to view and use your identity data.

###### Benefit:

* No lockout by third parties
* No loss of access due to bans or platform restrictions

**Example:** If a platform blocks you, your credentials still remain yours.

---

##### 4. Transparency

**Governance and algorithms must be transparent.**

Meaning:
The rules of the identity system should be open and understandable.

###### Benefit:

* No hidden data collection
* No secret decisions

**Example:** Clear rules about how credentials are verified.

---

##### 5. Persistence

**Identities must be valid for life.**

Meaning:
Your identity should be durable and not constantly reset.

###### Benefit:

* Protects against account loss
* Long-term continuity

**Example:** Your DID can remain yours long-term.

---

##### 6. Portability

**Identity data and services must be transferable.**

Meaning:
You should be able to move identity data between services.

###### Benefit:

* Use identity across platforms
* No vendor lock-in

**Example:** Use one credential in many apps.

---

##### 7. Interoperability

**Identities should be widely usable and license-free.**

Meaning:
Different systems should work together using open standards.

###### Benefit:

* No fragmentation
* No monopoly control

**Example:** A verifiable credential should work in many wallets.

---

##### 8. Consent

**Users must consent to the use of their identity.**

Meaning:
Nothing should be shared without your permission.

###### Benefit:

* Prevents unauthorized data collection
* Stops exploitation

**Example:** You approve every data-sharing request.

---

##### 9. Minimization

**Only minimum necessary data should be disclosed.**

Meaning:
Share only what is required, not everything.

###### Benefit:

* Better privacy
* Purpose-specific data use

**Example:** Prove you are over 18 without revealing birthdate.

---

##### 10. Protection

**Users’ rights must be respected.**

Meaning:
Identity systems should protect users from misuse.

###### Benefit:

* Prevents identity fraud
* Prevents identity abuse and discrimination

---

#### Overall Meaning

These principles define what **user-centered digital identity** should look like:

**From platform-controlled identity → to user-owned identity**

Instead of companies controlling your identity, SSI gives power back to the individual.

---

#### One-Line Summary

**SSI principles ensure that digital identity is private, portable, secure, transparent, and controlled by the user.**

---

- nice to make sure that all the really necessary information is used after the right of the need is not too perfect. Yeah.
- student: So **SSI is** a surely backend identity, right? So like in the front end you might see username and then at some point you add more users
and then you pick up the same name and then you say, hey, can all of my users to identify with who I might need to add state of birth. So you expand your definition in your use case of identity to maybe name and date of birth. But the SSI is like a purely backend identity that implements some kind of string to lock in your account or something, right? So it is nothing that someone who doesn't study computer science would ever see?
  - **No**, I literally just said that the wallet is loading and hopefully in a couple of months. So I mean what user is going to see is of course not the details as I'm going to explain here, but there is a user touching component to it. It's simply an identity management approach with an state of birth. So I think that they're very abstract place and you're going to see how it's technically also work for the feasible. But yeah, I don't know if I have to.
- student: So like **if I log in with my crypto SSI or with my Google account or the user is kind of the same experience**, it's just that the one that has no central institution and the other one does.
  - **Yeah, the objective is the same thing, to prove who I am and all of that is the same**.
- student: So it's like as a technical identity that is unique for each person.
  - Yeah. But **the user experience is also different**. I'm going to come to that.
- So these **principles** that were suggested by them are **not meant to be hard to move**. We also often see that **there are quite a lot of discussions** that are going on around these, but this kind of becomes a kind of reference point within the SSI community in general. That's stuff that is **referenced quite a lot in this organization's work**.
- Maybe something I should have said also earlier when we were talking about identities, we are often talking about **standards**. So that's something always to keep in mind. Without standards, you don't get **interoperability**. So we are going to always talk about standards as much as it makes sense and it's relevant and not too much in detail.
- Yeah. So the whole **idea** is that, as you can also kind of see, **each person should be the ultimate point of control over their own identity data**, and **they should decide what kind of information should be shared** between them, what should stay private and so on.
- That was the **motivation behind SSI**

## slide: "Decentralized Identifier (DID)"

- so let's have a look at the **first technical building block**.
- The **decentralized identifier** or **DID** as we use it.
- It's basically a new kind of a digital identifier that is designed to be only controlled by the subject itself, rather than issued by some kind of centralized provider.

## slide: "Requirements - Decentralized Identifier"

- And at its core **for the DID system to work**, there are some **requirements** that must be met. So how do I basically identify anyone in this whole system? That's what we're going to look at.
- So we need something that is **unique**, first of all, so I can really differentiate you from him and from her.
- And it should be **persistent**. So that's actually, again, the first back to the principles that I have talked before. So that I should make sure that my identity is pretty much there within my control.
- It should be **cryptographically verifiable**. So the ownership of the ID is proven by the use of a private key.
- It should be **resolvable**.
  - So the **information related to my identity**, this might be a set of public keys or some sort of metadata. They should be **accessible at all times**.
- **decentralized**:
  - And **DIDs can be generated and verified** across various systems **without relying on a single registry or organization**.
  - I would be a little careful with the **decentralized key** right there. It does **not** necessarily mean that it has to be in a **decentralized data structure like a BLT**, but the whole **structure of the DID standards** kind of **imply that** they should be decentralized so that we **don't rely on a single entity**.
  - student: I **still need some kind of authority that gives me my identity**, right? In case of your, like, I have a passport that's worth something because the government gave it to me and basically it says, okay, this is actually him, not someone else.
    - answer: **We can have various issuers for that**. And the **idea** is that **they should comply with a certain way of doing it**, which is pretty much **"the standard"** then, but it doesn't really care if it's coming from, I don't know, from Bonn municipality or Cologne municipality. So that level of decentralization you can think of, or you can think of it as Germany versus other countries.
- Yeah, so these are the **requirements** that we start with when we're looking at the DIDs.

## slide: "DID and DID Method"

- And what came out of it is actually something almost disappointingly simple.
- it's a simple **URI**.
  - When we talk about the **DID, it's a URI**.
- It's a structure that
  - always starts with `did` and then
  - **colon** and then we have
  - a **DID method**, which is basically connection of the identity and what kind of infrastructure I'm using and then
  - something that is the **DID method's specific identifier**, so a way to differentiate me within that infrastructure.
- So in the case of, for example, EBSI (Copilot: which is the European Blockchain Services Infrastructure, the DID method is `ebsi` and then we have a specific identifier that is related to me. So that's the structure of the DID.) we know that my DID can be resolved using the **blockchain infrastructure of the European Union**.
- I might also have a `did:web`, for example, which means that my information is posted on some kind of a **web server**. So that's also possible. It's actually unbelievably common.
- Or it can be laying on **Ethereum**. So we can check smart contracts to actually access my data.
- So basically the second part: the **DID method** is the **connection between identity and infrastructure, how my DID was created and managed**. It just tells me information about it or not really to me, but **to a resolver it tells the information where it should look for**.
  - There are certain technical regulations, so there should be
    - a `Create` operation, regardless of the ID, possible.
    - **Resolving** should be possible,
    - **updating** the information on my document should be possible and also
    - **deactivation** should also be possible, because it's also possible that my keys can get compromised and therefore I might need to actually deactivate the DID. So that doesn't really go against the persistence of it, but I am able to then clearly indicate that the DID is not valid anymore. And it should be transparently available to everyone that is trying to access this information.
- European Blockchain Services Infrastructure. That's the... **EBSI**. Sorry.

## slide: "W3C Open Standard Decentralized Identifiers (DIDs)"

- So, if you look at the W3C Open Standard, ~~and I think I put it so far wrong if I claimed, that this standard adopted in comparison to other standards are better.~~ So you might see that there are some systems that claim that they are W3C compatible. They are often referring to this, that they only have a DID method that allows you to do that.
- So **what does the** whole **standard** exactly **tell me?**
  - So I have a **DID**. That comes from a DID as I just showed in the previous slide.
  - And this DID is often resolving to a **DID document**, which has information in it that tells, okay,
    - what is the DID of this thing,
    - what type of different methods that I can use to access,
    - what are different methods that I can use to authorize this person, and so on.
  - And this DID is at the end referring to a **DID subject** about which we are making some statements.
    - So it might be **my DID**, for example, and anywhere where I use this, the DID is basically signaling that I am pulling something **about myself**,
    - but it might also be **about a machine**,
    - or it might be **about my child**, for example, and I might be still controlling it myself, that's also possible, which brings me to the **DID controller** itself.
  - So every DID has a **DID controller** at the same time that is taking care of this DID document, and the information that is inside it.
    - **My organization** might be the controller of my DID document, and they might **issue** me an **organizational DID**, for example, that I can only use within the work context.
- I just want to show you shortly. There is a [**universal resolver**](https://dev.uniresolver.io/) ([github](https://github.com/decentralized-identity/universal-resolver/)) from Decentralized Identity Foundation (DIF). Just to give you an idea what so **many different DID methods have already been created**, and this is not even... I'm not so sure how complete things I don't see, for example, some of the things that I have myself used in different projects already.

---

![DIF Logo](https://raw.githubusercontent.com/decentralized-identity/universal-resolver/master/docs/logo-dif.png)

### Universal Resolver

The Universal Resolver resolves Decentralized Identifiers (DIDs) across many different DID methods, based on the [W3C DID Core 1.0](https://www.w3.org/TR/did-core/) and [DID Resolution](https://w3c-ccg.github.io/did-resolution/) specifications. It is a work item of the [DIF Identifiers&Discovery Working Group](https://github.com/decentralized-identity/identifiers-discovery/).

See this [blog post](https://medium.com/decentralized-identity/a-universal-resolver-for-self-sovereign-identifiers-48e6b4a5cc3c) and this [webinar](https://ssimeetup.org/did-resolution-given-did-how-do-retrieve-document-markus-sabadello-webinar-13/) for an introduction.

See [https://dev.uniresolver.io/](https://dev.uniresolver.io/) for a DIF-hosted instance of the Universal Resolver that can be used for testing purposes. See [Docker Hub](https://hub.docker.com/u/universalresolver) for images.

---

### Gemini: "Decentralized Identity Foundation did methods examples"

The Decentralized Identity Foundation (DIF) and related W3C standards support various DID methods tailored for specific blockchains, ledgers, or use cases, including `did:key` for ephemeral identity, `did:peer` for pairwise interactions, and `did:web` for DNS-based identifiers. These methods facilitate decentralized, secure, and verifiable digital identities across different, often, non-interoperable systems. 

#### Examples of DID Methods

DIF discusses or lists several prominent DID methods that define how identities are created, resolved, and managed: 

- `did:key`: Typically used for self-contained, cryptographically verifiable identifiers that don't need a central registry.
- `did:peer`: Designed for private, pairwise interactions between two parties (e.g., Alice and Bob) without needing a public blockchain.
- `did:web`: Leverages existing domain name system (DNS) infrastructure to create DIDs.
- `did:ethr`: Based on the Ethereum blockchain.
- `did:btcr`: Based on the Bitcoin blockchain.
- `did:cheqd`: Uses the cheqd network.
- `did:dht`: Utilizes Mainline DHT for decentralized lookup.
- `did:jwk`: JSON Web Key method for cryptographic identity.
- `did:oyd`: "Own Your Data" method.
- `did:pkh`: Public Key Hash method for multi-chain identity. 

#### Context of DID Methods

- **Functionality**: DID methods are used to define the rules for creating, resolving, updating, and deactivating DIDs.
- **Registry/Storage**: Methods can be anchored to different systems, such as blockchains (e.g., Ethereum, Bitcoin, Sovrin), peer-to-peer networks, or other distributed ledgers.
- **Use Cases**: These methods allow for various use cases like verifiable credentials (e.g., digital diplomas, driver's licenses) and secure, decentralized authentication.
- **Development**: The DIF Identifiers & Discovery Working Group discusses and explores new and existing DID methods.

---

- So **decentralization** is whether we get in this way, **I can go to various places to generate my DID**,
- and **once I have a DID URL**, for example, here (on the page [https://dev.uniresolver.io/](https://dev.uniresolver.io/)) the DID URL `did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736`, **it means** that this **DID was posted in an Ethereum network**.
  - Have you guys learned about Ethereum already last week? It's a very known blockchain. That's the only thing I think you need to know about it. That's on the main blockchain network, and this is **my identifier on that network** to find the related information.
- And if you and I, for example, if you have a look at the **DID document** how it is looking like you can see (on [https://dev.uniresolver.io/](https://dev.uniresolver.io/) enter a DID URL in the field "did-url" (eg. `did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736`), then click on "Resolve" &rarr; "DID Document")
  - the **verification methods** (possibly multiple methods!) (`"verificationMethod"`, a list of items),
    - the **ID of each verification method** (`"id": "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736#controller"`),
    - so what kind of **key recovery method** (`"type": "EcdsaSecp256k1RecoveryMethod2020"`) is used for this,
    - who is the **controller of this identity** (`"controller": "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736"`),
    - it's a little bit **method-specific**, but **for blockchain** you **need an account ID** (`"blockchainAccountId"`), for example, and ~~it's an identity device~~. What is there? And **which standard** it is corresponding to.

---

### DID Document (DID URL: `did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736`)

```json
{
  "id": "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736",
  "verificationMethod": [
    {
      "id": "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736#controller",
      "type": "EcdsaSecp256k1RecoveryMethod2020",
      "controller": "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736",
      "blockchainAccountId": "eip155:1:0x3b0BC51Ab9De1e5B7B6E34E5b960285805C41736"
    }
  ],
  "authentication": [
    "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736#controller"
  ],
  "assertionMethod": [
    "did:ethr:mainnet:0x3b0bc51ab9de1e5b7b6e34e5b960285805c41736#controller"
  ],
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://w3id.org/security/suites/secp256k1recovery-2020/v2",
    "https://w3id.org/security/v3-unstable"
  ]
}
```

---

- student: The controller is the authority that generated this for you, right? Like in this case, that would be the German government, but it's obviously on the Ethereum chain.
  - answer: Yeah, so that's it. I mean, of course, the German government needs to say somewhere that, hey, this is me, you can find me there, but can be the German government, or can be me basically.
- student: Okay, thank you for the answer.

## slide: "DID Document - Decentralized Identifier"

- I already kind of showed this, so the **DID document** shows various information. It's quite a long list, and
  - the **only actually mandatory parts** are **context** and **ID**, it just says, what is this document supposed to look like, and the ID itself,
  - but very often you also see the **authentication methods** (= "Verification methods that can be used to authenticate the DID subject" (see slide), thus, authentication methods **ARE** verification methods: authentication methods **reference** verification methods) I would say,
  - you can see various lists of **public keys** that you can use for the verification later on.
  - There are certain **services**: the DID might also tell you, okay, for example,
    - for **credential issuance**, contact me at this address, and
    - for the **credential verification** operation, contact me at another address.
    - So, such services you can also define under the DID document (see [below](#chatgpt-did-documents-tell-information-associated-with-a-did-what-kind-of-services-can-be-defined-in-the-did-document)).

---

### ChatGPT: "what does the `@context` field in a DID document tell"

The **`@context`** field in a **DID Document** tells systems **how to interpret the terms and structure used in the document**.

It comes from **JSON-LD (JSON for Linked Data)** and provides semantic meaning for fields like:

* `id`
* `verificationMethod`
* `authentication`
* `service`

---

#### Simple Meaning

Without `@context`, a computer only sees labels like `"id"` or `"service"`.

With `@context`, the computer knows:

* what those terms mean
* which vocabulary/standard they belong to
* how to process them consistently across systems

So it acts like a **dictionary + schema reference**.

---

#### Example DID Document

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1"
  ],
  "id": "did:example:123",
  "authentication": [...],
  "service": [...]
}
```

###### Meaning here:

The context URL points to the World Wide Web Consortium DID standard vocabulary.

That tells software:

* `id` = decentralized identifier
* `authentication` = keys/methods used for login/auth
* `service` = endpoints related to the DID subject

---

#### Why It Matters

##### 1. Interoperability

Different wallets, apps, and verifiers interpret the DID document the same way.

##### 2. Machine Readability

Software can automatically understand relationships and terms.

##### 3. Extensibility

Additional contexts can be added for custom features.

Example:

```json
"@context": [
  "https://www.w3.org/ns/did/v1",
  "https://w3id.org/security/suites/ed25519-2020/v1"
]
```

This adds meaning for a specific cryptographic key type.

---

#### In One Sentence

**`@context` tells machines which vocabulary and semantics to use when reading a DID document.**

---

#### Easy Analogy

Think of `@context` like the **legend on a map**:

Without the legend, symbols are unclear.
With the legend, every symbol has meaning.

---

#### In DID Exams / Lectures

If asked: **What does `@context` in a DID document tell?**

Answer:

> It specifies the semantic context / vocabulary used so that document fields are interpreted according to the DID standard and related schemas.

---

### ChatGPT: "that means the context field specifies the JSON schema, right?"

Not exactly. **`@context` is not the same thing as a JSON Schema**, although both help software understand JSON documents.

#### Short answer

* **`@context` (JSON-LD)** specifies the **meaning/semantics of terms** in the document.
* **JSON Schema** specifies the **allowed structure and validation rules** of the document.

So `@context` tells you **what fields mean**, while schema tells you **what fields are permitted and what format they must follow**.

---

#### In a DID Document

Example:

```json id="2g39n9"
{
  "@context": "https://www.w3.org/ns/did/v1",
  "id": "did:example:123",
  "service": []
}
```

##### What `@context` does:

It says:

* `id` refers to the DID subject identifier
* `service` refers to service endpoints in DID vocabulary
* terms are interpreted according to the World Wide Web Consortium DID model

##### What it does **not** do:

It does **not** enforce:

* whether `service` must be an array
* whether `id` must match DID syntax
* whether a field is required
* max/min lengths
* data types in the schema-validation sense

Those are schema/validation concerns.

---

#### Analogy

Think of a form:

* **`@context`** = glossary explaining what each label means
* **JSON Schema** = rules saying which boxes are required and how to fill them in

---

#### Why People Confuse Them

Because both are metadata and both help machines process JSON. But they solve different problems:

| Feature                     | `@context` | JSON Schema |
| --------------------------- | ---------- | ----------- |
| Semantic meaning            | ✅          | ❌           |
| Validation rules            | ❌          | ✅           |
| Interoperability vocabulary | ✅          | ❌           |
| Required fields/types       | ❌          | ✅           |

---

#### Best precise statement

Instead of saying “`@context` specifies the JSON schema,” say:

> **`@context` specifies the vocabulary and semantic interpretation of the JSON terms, not the validation schema.**

---

### ChatGPT: "is the same true for the context fields of verifiable credentials"

Yes — **the same core idea applies to Verifiable Credentials (VCs)**.

In a **Verifiable Credential**, the **`@context`** field also comes from **JSON-LD** and is used to define the **semantic meaning of terms**, **not** to act as a JSON Schema.

---

#### Short Answer

For VCs, `@context` tells software:

* what terms like `credentialSubject`, `issuer`, `type`, or custom claims mean
* which vocabularies/standards those terms come from
* how to interpret the credential consistently across systems

It does **not** by itself validate structure like a schema would.

---

#### Example Verifiable Credential

```json id="53d0y6"
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1"
  ],
  "type": ["VerifiableCredential"],
  "issuer": "did:example:issuer123",
  "credentialSubject": {
    "id": "did:example:user456"
  }
}
```

###### Meaning of `@context`

This tells software to use the World Wide Web Consortium Verifiable Credentials vocabulary.

So terms like:

* `issuer`
* `credentialSubject`
* `proof`
* `type`

have standardized meanings.

---

#### Custom Claims Example

Suppose a university issues a diploma:

```json id="t59wq1"
"@context": [
  "https://www.w3.org/2018/credentials/v1",
  "https://example.edu/credentials/diploma/v1"
]
```

Now terms like:

* `degree`
* `graduationDate`
* `studentId`

can be semantically defined by the university context.

---

#### What It Does NOT Do

`@context` does **not** guarantee:

* required fields exist
* correct data types
* string length rules
* date formatting
* credential business logic

Those need:

* JSON Schema
* VC Data Model rules
* application validation
* cryptographic proof verification

---

#### DID vs VC Context

| Document Type         | Purpose of `@context`                                        |
| --------------------- | ------------------------------------------------------------ |
| DID Document          | Meaning of DID terms (`service`, `authentication`, etc.)     |
| Verifiable Credential | Meaning of credential terms (`issuer`, claims, proofs, etc.) |

Same mechanism, different vocabulary.

---

#### One-Line Summary

**Yes — in Verifiable Credentials, `@context` also defines semantics/vocabulary, not the validation schema.**

---

### ChatGPT: "DID Documents tell information associated with a DID: what kind of services can be defined in the DID document"

A **DID Document** (from the World Wide Web Consortium **DID specification**) describes how to interact with a **Decentralized Identifier (DID)**. It typically contains:

* Public keys / verification methods
* Authentication methods
* Service endpoints

The **`service`** section is especially important because it tells others **how to communicate or interact with the DID subject**. Each service entry usually has:

* `id`
* `type`
* `serviceEndpoint`

Below are the main **kinds of services commonly defined in DID documents**.

---

#### 1. DID Communication (DIDComm)

Used for **secure messaging between decentralized identities**.

* Enables encrypted peer-to-peer communication
* Used in many SSI frameworks

**Example service type**

```
type: DIDCommMessaging
```

Typical endpoint:

```
https://example.com/endpoint
```

Used for:

* sending verifiable credentials
* requesting proofs
* secure identity messaging

---

#### 2. Credential Issuance / Verification Services

Services for issuing or verifying **Verifiable Credentials** defined in the World Wide Web Consortium standard.

Examples:

```
type: CredentialService
type: VerifiableCredentialService
```

Used for:

* issuing credentials
* verifying credentials
* credential status checks

---

#### 3. Identity Hub / Data Storage

A service that provides **personal data storage controlled by the DID owner**.

Example:

```
type: IdentityHub
```

Often used to store:

* credentials
* profile information
* encrypted user data

Some implementations call this a **personal data vault**.

---

#### 4. Key Management or Cryptographic Services

Services related to **key operations or signing infrastructure**.

Example:

```
type: KeyManagementService
```

Used for:

* key rotation
* cryptographic operations
* hardware security modules

---

#### 5. Agent Services

Used in **SSI agents** that act on behalf of a DID.

Example:

```
type: AgentService
```

Agents may:

* negotiate credentials
* respond to authentication requests
* manage identity workflows

---

#### 6. Social / Profile Services

Optional services that link the DID to **public profile information**.

Example:

```
type: ProfileService
```

Endpoint may point to:

* profile metadata
* social links
* decentralized profile storage

---

#### 7. Payment or Financial Services

Some DID methods support **payment endpoints**.

Example:

```
type: PaymentService
```

Used for:

* cryptocurrency payments
* settlement endpoints
* digital wallet interaction

---

#### Example DID Document Service Entry

```json
{
  "service": [{
      "id": "#messaging",
      "type": "DIDCommMessaging",
      "serviceEndpoint": "https://example.com/didcomm"
    },
    {
      "id": "#hub",
      "type": "IdentityHub",
      "serviceEndpoint": "https://hub.example.com"
    }
  ]
}
```

---

✅ **Key idea:**
The DID document does **not contain the services themselves** — it only provides **discoverable endpoints and types** so other parties know **how to interact with the DID subject**.

---

💡 If you want, I can also explain:

* the **exact structure of a DID Document (all fields)**
* the **difference between verification methods and services**
* or **real DID document examples (did:ethr, did:key, did:web)** used in Web3 and SSI.

---

## slide: "Verifiable Credential (VC)"

- So, the **second building block of SSI** is **"Verifiable Credentials"**, or **often called "VC"**.
- So, the **DIDs** are basically about **who you are**, and the **VCs** are more about **what you can prove about yourself**, so **what kind of claims I have**.
  - So, if the **DID** is pretty much **my name**, then I would say that the **ID card** is a **verifiable credential**.
- So, it's **a digitally signed statement** made **by an issuer** about a subject.
  - So, for example, that someone holds a university degree, or a driver's license, or some kind of a certification,
- and just like a physical credential, it actually **names the holder** and **states what is exactly being attested for**.
  - So, in an **ID document**, for example, **my address** is something that the **government is attesting** that they know about me, or that it's true,
- and **it includes a signature** that guarantees the authenticity.
- And, of course, the **difference to a physical credential** is that
  - they are **not checking out the issuer's database**,
  - but all of these **information is cryptographically verifiable**. So, we are trying to do this digitally.
- So, formally, we have **three identities**, and when we're talking about an SSI system, it's often these three identities that I talked about:
  - We have an **issuer** who creates and signs this credential, eg. the government.
  - We have a **holder** who stores these credentials in their wallet, eg. me.
  - The **verifier** who checks the credential when the holder actually presents it, eg. supermarket.
    - And the difference is that the **verification can happen instantly**, so I don't need to contact any people to make sure that this is really true. Eg. an employer doesn't need to contact the university to make sure that my credential is valid. They can just check the signature on that.
- And various **claims inside a credential** can basically express different things.
  - So, I can have **attributes** about the object of the verifiable credential, eg. age, qualification, or health status.
  - It might symbolize **relationships**, eg. I am an employee of Fraunhofer FIT, so that I'm allowed to prove this relationship.
  - Or it might also express certain **permissions**, so I am allowed to, for example, log into some webpage with the admin role. So, **role membership** (a CS term) and **access rights** can also be expressed in a verifiable credential format.

## slide: "Examples - Verifiable Credential"

- That's what we already talked about what's with driver's licenses, and educational certificates, but
- this (on the right side of the slide) is an example of what the verifiable credential is looking like. **A VC is basically a JSON**, like this version (format) of JSON, if you look at **W3C's standard**, that's at least the case. If you look at **different standards**, you might find different things, but the gist of it is at least the same.
- It has some kind of **context information** (`@context`), so how this document is supposed to look like, like what are the mandatory fields and so on, so that I don't get something that's looking completely differently and claims to be a verifiable credential.
- I have an **ID** (`id`) for this specific credential.
- I might have a **type** (`type`).
- I definitely have an **issuer** (`issuer`), which is also described with the **DID (of the issuer)**.
- The **issuance date** (`issuanceDate`).
- We have an **expiry date** (`expiryDate`). It's not necessarily mandatory, but we can have it.
- And we have **credential subject** (`credentialSubject`), so this is basically a **subject about which we are making claims** about, and that we are also using the **DID of that subject** (on the slide: the DID of the student),
  - so **if it's about me**, then you might have **my DID**,
  - but you might also **assign the DIDs to different machines** and **issue credentials to those**.
  - So as long as I mention what the credential subject is, it's fine.
- And then we have a **cryptographic proof** (`proof`) basically about the whole thing.

## slide: "Requirements - Verifiable Credential"

- So this was a standard, but overall what we expect from a verifiable credential is that
  - it must be **tamper-proof**. So it must be digitally signed, so any manipulation to the credential must end up in a negative verification outcome.
  - It needs to be **decentrally verifiable**, so the authenticity and origin can be checked by looking at the issuer's public key.
  - And it must be of course **interoperable** so that I pretty much need the ecosystem that this thing is working, so it needs to be accepted everywhere so that I actually use this.
  - And it needs to be of course **machine readable**.

## slide: "Issuing a Verifiable Credential"

---

### ChatGPT: "meaning"

This describes the **issuance process of a Verifiable Credential (VC)** in a **Self-Sovereign Identity (SSI)** system.

A **Verifiable Credential** is a **digitally signed credential** (like a digital ID card, diploma, driver’s license, membership card, etc.) that can be stored by the user and later presented to others.

---

#### Meaning of Each Step

##### 1. The Holder sends a request to the issuer

The **Holder** is the person or organization that wants the credential.

They ask an **Issuer** to receive a credential.

###### Example:

* A student requests a digital diploma from a university
* A citizen requests a digital ID credential
* An employee requests a work credential

➡️ Similar to applying for a physical certificate.

---

##### 2. The Issuer verifies eligibility

The **Issuer** checks whether the applicant qualifies.

This may involve checking records or documents.

###### Example:

* University confirms graduation
* Government verifies identity
* Employer verifies employment status

➡️ Only authorized people receive valid credentials.

---

##### 3. The Issuer creates and signs the VC

After approval, the issuer creates the credential containing statements (claims) such as:

* Name
* Degree earned
* Age over 18
* Employment status

Then the issuer **digitally signs** it using its **private key**.

###### Why sign it?

The signature proves:

* who issued it
* that it has not been altered
* that it is authentic

➡️ Like an official stamp, but cryptographic.

---

##### 4. The VC is sent to the Holder and stored in wallet

The credential is securely transferred to the holder.

The holder stores it in an **SSI wallet app**.

Examples:

* Mobile identity wallet
* Browser wallet
* Enterprise credential wallet

➡️ The user now controls the credential.

---

#### Overall Meaning

Instead of a company storing all your identity data centrally, the issuer gives **you** a signed credential that **you keep** and can later share when needed.

This is the core SSI idea:

**Issuer issues → Holder stores → Verifier checks**

---

#### Real Example

###### Digital University Diploma

1. Student requests diploma VC
2. University checks graduation records
3. University signs diploma VC
4. Student stores it in wallet

Later:

* Employer asks for proof of degree
* Student presents VC
* Employer verifies signature

---

#### One-Line Summary

**A Verifiable Credential is issued after eligibility checks, digitally signed by the issuer, and stored by the user in their wallet for future proof.**

---

- student: Sorry, would it be something like the government issues me credentials, signs it with their private key, so anybody can verify that it came from the government, and then I have a private key and I also sign over my own credentials, which means I can authenticate myself to a supermarket, a bank, a university, etc. So would it be something like that?
  - answer: Yeah. **At least the first part is correct**. How do I **issue a verifiable credential?** So the holder basically sends a request to the issuer says, okay, I need something from you, I need you to make an attestation about myself. The issuer of course needs to check within their own system, is this person really a warranty of this credential, let's say. And then the issuer creates a VC that doesn't say when that's signed, that's at the private key, like you said. And then the VC is securely sent to the holder and stored in their SSI holder. So this part is actually quite interesting and annoyingly complicated. Just a question, just to brainstorm a little bit. **How do you think a credential can be sent securely to someone** in a digital setting?
    - student: Sorry, usually you have a third party involved, right?
      - prof: What do you mean "third party"?
    - student: So like, apart from RSA, for example, which just hopes that logarithms solving is hard, is that usually you have like, when you're talking about certificates, which you use for signing, you have a institution which issues the certificate, and then you have the person giving another person the credential, and then this person has to contact the issuer to check whether the certificate is right, and then you have the third party coming in. No?
      - prof: Well, we are trying to run without third parties.
    - student: Yeah, this is also like, my question here, this is how I learned signing usually works. And this needs a third party, so...
      - prof: We have a signature already, and we already... So let's say that you are in contact with Fraunhofer FIT. And you're going to receive a **verifiable credential** so that you can **prove that you are an employee to FIT**. We already talked among each other, there is a communication channel that we trust, let's say email. That's a separate discussion, but let's say email. You are receiving a signed and encrypted emails from me, that's definitely FIT. And I have created this already, so I am telling you that I have your VC, I just need to **somehow transport this JSON (that I have shown) to you**, so that you can save it somewhere. And on the way you need to make sure that it's a securely saved, and that I am still the person that I'm claiming I am, and so on.
    - student: I think it was man of exchange or something, like I write prime numbers once and one time, but I forgot how it works. 
      - prof: You need to put it in your place. 
    - student: I don't know what you want to hear, but I think that the VC is going to be my part, or I can be my public key, and you can put your data in your place.
      - prof: I mean, there is definitely some cryptographic stuff involved in it,
  - prof: but I guess I am asking more on the **what is the user's experience** in that. But no need to actually make that longer, I just want to point out that this is actually an interesting problem. Surprise, surprise, **there are standards for that (for sending a credential securely to someone)**. So if you comply with the standard, you are getting in securely, and at the moment there are different standards that are competing against each other. But there is one that seems to be the winning one, or the more commonly accepted one. This is a little problematic to explain, because the whole stuff that I am explaining on SSI is still evolving at the moment. It is quite a recent thing that we are talking about. ~~That is definitely something that I have been asking for years.~~
- I am just going to show you a link here now. It is from a project that we implemented. It finished recently. And there we got some nice amount of funding from a blockchain infrastructure for wider hashgraph association, that we investigate different use cases for their infrastructure.
  - student: And that is the moment I wanted to say, that we must consider the last mention.
    - prof: Sure. I am actually not really going to go too much into details. To be honest, I also know the details quite roughly, the parts that I implemented most.
- So, what was I saying? The project. So we got a little bit of a chance to basically use this as a playground for ourselves, and have a look at the interesting to implement the SSI stuff inside our own organization. So what can we do with the SSI in general?
  - We can issue, for example, employee credentials, and I already gave this as an example away.
  - But then I can get two of all the keys that we are using inside our institute that are extremely annoying for us. And instead we can use **credentials to actually access our own rooms**. But not only that, we can also say that someone is allowed to enter a room, only if they prove that they have taken certain education, so certain training, and they are capable of using the device in that room. So, not only authorization based approach, but also checking the capability if this person is really capable or secure enough to use this device in that room. And meanwhile, we don't have to create any databases about the users. It's only enough that they show us the credentials that were issued by us.
  - So one of the use cases was basically **onboarding** a user. So when they first come, how do we exactly issue these credentials online? We need for the demonstration this video, so I'm going to show you that. And in our use case, it basically starts with a secretary that is also using our system by logging in to it using our own credentials. So we do not know who this person is specifically, but we know that they have a secretary role because we got it from their credential. So we don't have to check specifically that Maria is employed as a secretary, but we can check already in the credentials. And this person has access to various onboarding services inside our organization. That's a proof of work. So, ignore the UI part. So the only point is to just show that it is working. We had a huge ecosystem that wasn't just about SSI. But the first basically point starts with "secretary registering user". So the organization first needs to make sure that this person is really in some way a employee. So this is a pre-filled form. We call it visa employee and it has a certain contract date, certain roles and we can define certain training that this person needs to provide. And then register the employee. And that this employee gets, or the secretary gets a link that they need to share with the employee. So in our case we set that we are already in contact with this person. We need to add up everything also to existing workflows a little bit, which is sometimes far away from the optimal way of doing it. So we said that this would be shared with an email. So as soon as the person gets the email, they need to enter a personal code, which consists of their name and surname, and lastly the birth date. And afterwards what's happening is that they are basically seeing their user profile and meanwhile in the background we are generating a DID specifically for this user. An **organizational DID** we call it, because one can also theoretically have their own private one. And once we are talking about a wallet, you will have a private DID, but we want to specifically extract the organizational one, so we are assigning ourselves. That's hosted in Hedera, this is all the IDs groups that I showed you. But the point of showing this here is basically the person can double check this information before it gets a credential. And then the **QR code** is shown. So this is the part I was actually asking about in the "securely sending" thing. The standard **"OpenID"** for VC is basically starting with a QR code, where the issuer says, connect me at this address with some kind of a pre-authorization code that's generated for that user. So that code is interpreted by the wallet, and there's an exchange that's going on with the issuer on the back end, so if you don't know anything about it, this is the wallet by the way. And as soon as the issuer can track exactly who I am, then the connection is signed and then sent. But from the user's perspective, what it's only seeing is that it's **scanning a QR code**. It might potentially fill in required details, that's not a must, in our case that's the requirement from Hedera, we have to go around it. And then we see various issuers of what's happening in between, at least in the related parts, and actually there's issues that take some time, because we don't have time until the wallet really decays it. But there's a secure connection going on, and you can see your interaction here, between the issuer and the holder, and it's asking, this has been issued for you, would you like to really save it in your wallet? And as soon as the person actually gets that, this information is shown again, and can be accessed at any time. So at this point on, as the user, I am responsible for this data. And I'm responsible for also sharing, partly or fully all the details with other people, but the issuer is not really involved anymore with the details of this data. Yeah.
    - student: How is this different than the idea of having **digital signatures**, for example, there's like PGP for e-mails, right? I go somewhere, it issues me a private key, I put it on everything, so for example, if I want to get confirmed that I have a master's here, right? Honor, I don't know, diploma, how would put their private signature, and then they send it to me, and then whoever wants to see, you know, was I really a student here, they just use it. I feel the idea is still somehow the same.
      - prof: It is. It's actually nothing so different because you just put a cover on it. But what we have here is pretty much someone that I trust, and hopefully the verifier also trusts, basically, **digitally signing** some information. I think it becomes a little bit more clear when you want to verify this information, maybe there comes some sort of factors. But at this point, this is nothing more than some fancy JSON that is signed. And of course, a little bit more sophisticated is just identifying people. So I can actually check various systems, I can find their public key, and then come back here and look, is this really the issuer? Is everything out there fitting? Yeah.
  - So this is where we get the credential.
  - Yeah. And when we want to actually use this credential to start an application, then the process is pretty similar. Another **standard** for it is **OpenID for VP**. And it's, again, **simply scanning a QR code from a user's perspective**. On the backgrounds, there are a lot happening, but I will explain it in a minute. But here you see that before any kind of information is shared with the verifier, it's showing you exactly what this one person wants. It wants a credential of type ID card. Here, there are various votes because they make it more fancy and more visible, I also haven't got it, yeah. So but the idea is that the concept of user is taken, they're showing them the information that are asked from them, yeah.
    - student1: Before, why did we need to have the DID of the organization? I thought it would be like, you're just adding one more DID, right? Because you have, let's say you have your own DID, and then you provide that to verify that's you when you're making a verifiable credential. But why would they generate a new DID?
      - prof: In this project?
    - student1: Yeah, sure. In general, I mean.
      - prof: We needed to do this because SSI is not widely adopted. So we didn't want to go to our own colleagues and say, yeah, generate a DID for us and enter the DID while doing that. I think that would also be possible. And it had also something to do with the key management because we, so for the work-related stuff, we were providing certain funds to those accounts that was coming from the organization and we didn't want the user to have access to those keys just out of organization regulations let's say. So another organization could do it differently. I don't think that was the most optimal thing, but out of the requirements we had that was how we did it. So it is possible.
    - student2: So one DID can have information for multiple issuers?
      - prof: There can be multiple credentials issued to the same DID subject.
    - student2: Okay. So the government could issue my name and something and the university could also issue my degree.
      - prof: Exactly.
    - student2: Okay.
- So and **wallet**, and what you are **going to be managing is information that you get**. Sorry, I actually jumped. Yeah, so I will show you in a minute. Here, just to wrap it up, here I can select from also a bid. In our case, the only one connection is the instance of obvious. But if I had multiple connections, then **I can also see from which and what kind of information is going out**. So the constant is there as much as the designers all know it.
  - student: Wouldn't it be very important in such a system that people keep their private keys offline and very safe, sort of like a quite hardware wallet, because anyone who can get their private key could then impersonate them.
    - prof: Yeah, that's an important point. I mean, **wallet's main responsibility is pretty much keeping it safe**. So we didn't really deal with it because the wallet wasn't implemented by us. Very often it's... **We don't really want to implement the wallet**. I can promise you that it's going to be complicated. But you can see at least what kind of mechanisms that they are using in their wallet. It's often Open Source information somewhere.
  - student: One thing to know, the thing about it's, for example, one of the bad things personally for me with this multiple **identity providers** is that, for example, Facebook has access to my data and they have a lot of data that they can do whatever and sell it. And I don't exactly precisely understand how, for example, with this we're preventing them to do... They ask for some data, we do this identification process and they get our data. **How can we, like, with this method, make sure that they are not going to do anything else with our data? Like, how does this prevent it?**
    - prof: That's another point. That's actually **one of the criticisms that I find very valid**. That you **don't have any control over what the verifier might do with the data that you share**. But you are in control of... Or **you are the one that decides what information you are sharing**. One suggestion often is that you can theoretically use a separate DID for interacting with different parties to avoid profiling. Might help, I don't know how realistic that is to be honest, that I would expect that the wallet might have something like this and that is the user doesn't know about it at all, then maybe. But still, if you have a verifier that is really asking a lot of information from you and if you don't care about giving it, there's nothing that you can do, at least not with this application
  - student: Like, the issuer doesn't know how often and to whom I use my credentials.
    - prof: No.
  - student: So, if Facebook gives me credentials that I have an account...
    - prof: Then Facebook knows that I'm logging in with it.
  - student: Then Facebook knows what I put into Facebook, but it doesn't know if I use it to log in 20 different applications.
    - prof: Exactly.
  - student: And that's the criticism of the current state and this is solved by the system, right?
    - prof: This is tried to be solved with the system, but it heavily depends also on how you really build these features into your system. Because theoretically, you can implement a system for **revocation checks** for credentials, because that's something I need to check as well for this credential involved. Let's say there's a web server, then there's absolutely nothing hindering me from checking out which information was checked for revocation. There are also various approaches for that, like there are **privacy preserving approaches**, but every small part of this whole puzzle is by itself a big problem actually, that the whole problem is trying to solve, but that does not necessarily mean that every single SSI approach is perfectly adhering to all these principles. It's just the guiding principles, what I showed at the beginning.
- Yeah, so sharing with it, and that's pretty much it. **The user's experience is scanning QR codes, selecting something and sending it**.
- In **our onboarding application**, that's what we called it, we had also various things that we needed to do.
- In today's world **at Fraunhofer FIT**, there's a non-research institute, let's say it, when new employees onboard, we give them a paper, which has basically the contact person that they need to go and for what exactly, like to pick up your laptop or to do some trainings and get information about some trainings, and you need to find these people in their offices. Before Corona, that was a bit more easier. Now after Corona, it's people doing hardly home offices, it's even a bigger problem. So let's say you find this person in place and got it, and then you need to get a signature, and at the end you're recommended to make a copy of the document for yourself. So you can see it's not really a very advanced digital way of doing it, so our approach was more trying to do it a little more digital.
- For me, **when I did it as a student, it took me about two days**, I think, to get all the signatures. And from this perspective, our biggest argument was that theoretically one can do this before actually coming to their first working day. So you're not going to be using two working days. From an employee's perspective, I don't know, it's really a loss, and I'm not spending two days doing nothing, but yeah.
- student: Is the DID encoded in the secret, some of the government gives me credentials, could I use the same secret in another DID?
  - prof: What do you mean "secret"?
- student: I use my credentials with my DID, so I want to log in somewhere, and so the where I log in knows my DID, and if I log in somewhere else with the same DID, that could be linked. If I want to use my credentials with a new identity, could I use the same credentials that I had before, or...
  - prof: **The credentials are tied to the DID itself**.
- student: I would **need to get new credentials issued for a new identity?**
  - prof: **If you want multiple copies of it, yeah**, you would need to. But very often, we are not really specifically checking for the DID values, but for profiling you can do that. But what's more important is that it's coming from a trusted issuer.
- student: Let's say I want to have five Google accounts and use a five-time something, but Google wants me to use credentials that I live in Germany and I have a name and something. Could I use my credentials from the government that I have a name, five times with new identities toward Google. Tied to the DID that I used to get this from the government?
  - prof: I mean, if you are being issued a credentials, I guess it would be tied to a single DID. I'm not sure if you have any questions or anything, but...
- student: I want to create an account for Google. Google says you need to be a real person. So it gives me credentials, you're a real person. I give them my credentials from the government. I'm a real person. Could I use the same credentials with another DID and say I'm a new person? And then just telling you I have a real name.
  - prof: It depends on how these credentials structures are generated. So option here, I think this ID is mandatory, if I'm not entirely wrong.
- student: But if I replaced the ID, would the proof still work.
  - prof: You can't just by yourself issue, because you're not doing time-tied issuer. So, you need to go to the issuer and tell them, hey, I have a new DID, so issue my credential for this DID, and make the old one invalid. Yeah. So with the signatures, it's actually... You probably can't just change the value there. But you can create a different DID for the interaction only. But to be honest, I don't really know the details of it, I never had any working example.
- student: I was wondering, could we have this **problem of being tracked**, because we only have one DID, we solve by having one central, one authority. The government gives you a DID, and then you have another issuer who, given you have this DID from the government, gives you a new DID, which is also trusted, because they only give it to you if you have the proven one. And then this issuer gives you a DID for every single service you use, basically. So you cannot be tracked across several services. 
  - prof: You can implement a service that utilizes an ID card credential to get the information, and then issue your automatic ID information. There's nothing hindering that, but it's not enforced that there is really this one authority, that you might be trusting your government, but that might not be the case for everyone. 
- So I just think a new employee, yeah,
- in this video, I just want to show further, that we also just said, okay, theoretically from training, so you can also get credentials. I'm not just jumping because it's pretty much the same thing, you're a scanning a QR code, and so on. We issue do process, we issue training certificates. I mean, that's some of the stuff that's something that we can test as well.

## slide: "Verifiable Presentation (VP)"

- So now the question comes to what we have, kind of actually already spoiled. We also have **verifiable presentations**. So I have this credential that I'm holding, and the next question is that, okay, but how do I present them safely? And this is where we have the concept, it's basically the bridge between the user and the verifier, and the way that the holder can basically selectively share information while also keeping control.
- Now, a little more precisely, **verifiable presentation**, can be generated out of one or more credentials.
  - So I got a credential from the government and from my university, theoretically I can pack them together in one presentation, so I don't have to do it multiple times.
  - But I can also say that out of this credential I only want to share the information that I'm over 18. That's also possible. So that's where the verifiable presentation is **different from** the **verifiable credential**. I'm not sharing the whole information. I don't have to share the whole information. I can decide. And the holder basically creates it based on the verifier's request on what kind of information they need from you, if you agree with it, and then sign it with their private key. So it's, again, pretty much signature, but of course you need to make sure that the signature of the issuer is still trackable, combining signatures from different JSONs and then putting it in a more trusted packet is also another technical challenge, but there are some approaches for that.
- But two things about the verifiable presentation that are important is that
  - it should be possible to **selectively disclose information**. So I need to be able to decide what I'm sharing.
  - And the second thing which is very interesting is that there can be also **zero-knowledge proofs for yes-or-no responses**. So in order to share that I am over 18 I don't have to necessarily give out my date of birth. So there are some models that might also enable this.
    - Technically, I know some ~~roles~~ that directly support it, but they are not compatible with W3C standards as they are completely different. So according to my knowledge, W3C compatible zero-knowledge proofs in practice, I don't know anything like that yet. I know that there's work on it that is being done.
  - Yeah, but that's pretty much the two important things about the verifiable presentation signatures

## slide: "Verifying a Verifiable Presentation"

- the verifier sends a request to the holder and
- the holder receives this request and checks if they have those credentials or that information and
- agrees to provide the data and sends it back to the verifier as a verifiable presentation.
- A thing that made it a lot more clear for me when I was actually learning the concept myself was that what does verifier exactly check?
  - So first of all, we are having a look at signatures. The authenticity of the data is still there because it's not manipulated at all that we can do by looking at these DIDs, resolve these DIDs, get the document, look at the public keys, check the signatures, do they fit? Do they not fit? Then, I have to check the identity of the issuer. I think one of the biggest things about this whole SSI thing is for me at least that the whole concept holds as long as verifier has trust on some issuer. So if I'm not trusting the issuer at the beginning, this whole thing doesn't mean anything at all. I need to know for sure that, first of all, the government is an entity, for example, that I trust. And the second is that they indeed publish these DIDs somewhere, so that really belongs to them, I know it. Only then, the whole thing makes sense. However, if I'm not already trusting University X from another country, then this whole thing doesn't make any sense. All of this also came with the different approaches, like can we create certain registries where there are trusted data issuers, various approaches exist for that too. If I'm not terribly wrong, I think, within the European Digital Identity Infrastructure, there is a similar approach as well to identify these from different countries, which issuers are really trusted, ie. to see whether this DID is really someone that I can trust.
- Yeah, the identity of the issuer, and that we also check the holder was indeed the person, or the signature of the holder match the DID, the DID's very important public key, and then checking the signature, and that the credential has been valid and not revoked yet by the issuer. It is possible to revoke a credential from the issuer's side, that's why I said that it's possible to also conduct a revocation registry, where you can also theoretically track whether this credential has been accessed, because at this point the verifier needs to check if it has been revoked. So with that, theoretically the issuer can get that information, but as I said, there are some types of various approaches for that, that doesn't say anything about what kind of credential is really being checked, but the results are valid results.
- And also very nice is that I can issue a credential, for example, in our project. We can issue an employment credential, and already put the end of the contract date, and afterwards I don't have to worry about it, because the information is already there, it's invalid, and then it will always return negative results when the verifier wants to verify it, so I don't have to actually deal with off-boarding of my employee. If their contract is extended, that's a whole other story, then you need to maybe issue another credential, or extend it manually, but it allows to actually already restrict certain allowances by a certain time period.
- And of course, what's not here is that, as a verifier, I also need to check whether it conforms to my requirements. Like, you are supposed to be over 18, is a better example, but like, for FIT my requirements are that they allow access to some kind of service.

## slide: "SSI Wallet"

- Now the part **wallet**. It's for me a little bit like a damned topic, which I got to know when I actually first started implementing something on this point, because without a proper wallet, whatever you do means absolutely nothing, because if the user is confused, if the user finds it unattractive, then anything fancy that you do on the background doesn't mean anything, and if the wallet is not functioning well enough, you can't ~~press~~ anything at all. So that's also quite annoying, but the wallet is pretty much the part that is facing the user.

## slide: "SSI Wallet und Digital Agent"

- And this is pretty much the digital equivalent of your wallet, physical wallet. I have my cards, I have my identity cards, I have my university cards, the same thing, that I already kind of showed. It manages pictographic material, my keys, and it also ensures that the interactions with the others are staying secure and privacy-preserving.
- So it's, on the one hand, has a **user interface** and **data storage** facilities.
  - It should be able to, of course, create a **verifiable presentation** using selective disclosure of attributes.
  - It needs to **check for the expiration** of the credentials and for the user, for the user finance perspective,
  - **Backup and Recovery** is a whole another topic, that's also open for discussion, but in an optimal way, it should be possible. But as long as you start thinking about backup and recovery, there come some, again, centralized entities that manage it for you so that you can easily recover them. So again, it comes to the usability vs. security issue. On the other hand, if you don't have any of those centralized approaches, then you might... if you lose your keys and you're fine with it.
- Yeah, and all of these wallets or entities, actually including issuer and verifier, is basically wrapped around the concept of **digital agents**, at least that's how we're calling them, and
  - that's a side ecosystem that's responsible for **receiving requests and exchanging credentials** for all these standards that I mentioned like OpenID, for VC and VP, they need to support these standards,
  - and it's often a **secure, asynchronous communication** which is also a complicated topic to handle.
  - And you need to be able to also do **connections management**. But often wallets ask if I trust this person with this DID and so that once you say, yeah, I know this person, but I don't have to answer this question all the time. So I can just add certain people to "Trusted DIDs" for issuance or for verification. Some wallets support this, some wallets don't. It depends.

## slide: "Verifiable Data Registry"

- And all of these things that I have already explained depends on a **shared trust layer** which we call **"Verifiable Data Registry"**.
- So it's not really a database where you are saving all the identity information, but it's rather, we need somewhere to store the ID documents.
- Credentials, they sometimes follow schemas so that every university indeed issue the same format of credential or follow certain schemas. These need to be stored somewhere and of course this needs to be secure as well.
- **Presentation requests** also have something like a **schema**, let's say, that we can define how a presentation should look like already or **revocation registries**, for example. So such stuff still need some kind of a **data registry** that is reliable, but also preferably decentralized or distributed in some manner. It should be available all the time so that I can continue to operate within this ecosystem and the integrity of this should be really made sure and this is actually where the **blockchain** comes in.
- So the invention or the rise of **blockchain** kind of kicked this whole thing a little bit forward because there were some people that said, okay, this is actually an awesome place where we can save all this information, at least for the public ones, that we really want to save. And this is how I also a little bit triggered the whole developments in the SSI.
- On the other hand, today, you don't have to necessarily have blockchain to have any of these SSI concepts.
  - You can say that I'm running a web server and hashing it every day and then checking if the hash is still fitting for the integrity. Whatever, like, such things also exists. It doesn't have to be a DLT, but a DLT is indeed very suitable, especially for the issuers, like for the governments and so on, to actually say "this is me", it is a very good infrastructure because I hope you know that too, from the last week, that the data that goes in blockchain is **immutable** and cannot be changed. That's why it's actually a good platform for it.

## slide: "SSI Frameworks"

- If you're interested in the topic, this is more intended to give you starting points, like if you want to implement some stuff and so on, or if you want to look how all these things are working.
- **Hyperledger** is following a complete different approach. It's called **AnonCreds**. It has nothing to do with any of the standards I have followed, but they are actually quite privacy-preserving. It's interesting. And this is where I also got to learn the concepts quite nice.
- There is frameworks. It's just an idea for you if you want to have a look at it practically.

## slide: "PKI versus SSI"

- The question that comes often is basically the difference between **public key infrastructure and SSI**.
- Basically, the **trust model** is quite different. So we have certificate authorities with public key infrastructure. On the other hand, in a decentralized, especially decentralized SSI network trust is created by the properties of the ledger and also the cryptographic proofs that we've been talking about.
- There are very strict **certificate formats** for PKI and the VC parts schemes can be quite flexible. It depends on the use case.
- PKI has a strict **hierarchy**, top-down control. On the other hand, if you look at SSI, anyone can be an issuer and verifier. There is no real separation for different identities.
- **Distribution and use** for the PKI is quite accepted. At the moment when we talk about SSI to people that do not really know much about it, it's not perceived as secure as PKI, although there's actually quite strong background to it.

## slide: "Challenges"

- So, let's wrap it up a little bit. I have already kind of seen all this.
- **Interoperability** is quite a big problem. There are various standards that are racing with each other. There are various frameworks. I can promise you, if you really want to start implementing something like that, this thing is extremely complicated when you don't know exactly what standard you are adhering to and finding this information itself is also quite annoyingly hard. It's getting better though.
- **Wallets** itself is causing a problem, as I said. It's availability, security and especially the usability of the wallets definitely require some further work. If you're interested in better usability directions, you can have a look at it, I think there's really missing informing literature. It's not really tested though, as far as I understand.
- **Building trust and acceptance** by authorities, companies and users is of course a hard thing. How are you going to exactly convince? We tried it within Fraunhofer FIT, we demonstrated our own product. We had some stakeholders and they were just constantly talking about PKI infrastructure and how trusted it was because it was vetted by some German security ministry, I don't remember what kind of organization it was. So, this whole thing needs to be adopted. And on the direction to the adoption, business models are a little bit problematic. So, it wasn't exactly tempting to be an issuer. Who are the drivers of this whole concept? How do you exactly start using these? On the other hand, with the early wallet, there's a little bit of a promise. Who knows if the government says, okay, I'll use this structures from now on, maybe the companies also adhere with it.
- Compliance with various... I think it's a relatively positive step in the direction of compliance with privacy rules. But as I explained, it doesn't mean necessarily that a SSI system is really fully adhering to all of these principles. A standardization is needed at the national and international level so that I know an education credential that is issued by Germany is really accepted.

# After Lecture

Doing it digitally
But build the moves in my part is a little bit
recent I would say
essentially the blockchain and
that's when I put certain conditions and the money that always
Stays like this you can update this you can remove these conditions after certain rounds or after a certain amount of transactions
I can yeah change it and
When I first started with this topic I
Talk to myself as I was around the same time that I was also hearing about this SSI thing that it would be actually very useful when
Users could provide this information these proofs that they will comply with the rules
Because otherwise where do you get this information of you know the boots that I'm buying a buying
books and
Food but not
Games I had some examples today. I don't have to unfortunately
Jump around it
Just want to give you a little bit of an idea what are the confused cases
Well, I'm looking at the donations. I can say I want to give certain amount of money to people below 18 years old
And I want this money to be used only for computer accessories because I have a personal bond to this
I
Said I can also say that after my debt
I want this money to go definitely to this person and
Can be only spent for these these purposes
For project management, you can say that you can do this with the money and already assigned certain categories to it
that's
More or less it
Yeah
Now the important thing for me is
currently I'm running a survey for my PhD thesis and I would really appreciate if you could take part in it
because
I really need a lot of input.
Right now it's only the SSIs, so I would really appreciat it. It's available in two languages
