---
title: "Web3 and Distributed Ledger Technology - Ethereum and Smart Contracts (Part 2)"
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

So, the thing, last time I reached this slide here, about the **Ethereum account**.

The idea of lecture today is basically I will finish this presentation, I will try to explain how the accounts work, how the smart contracts work,
and then in the end we can deploy one or two smart contracts together depending on how much time we have, so that I basically give you an idea how it works,
if you want to deploy your own smart contract, what can you do, how can you do it easily, so that you don't waste some time.

## Recap

So, yeah, just a small **recap** on the last presentation, last week, on this slide here where we left.

## slide: Ethereum accounts

You see here that we get the externally-owned account and the smart contract, again now we are going to the smart contract,
but before we jump to the smart contract, just to remember, because this is also the **exam relevant question**, like what's the difference between externally-owned account and smart contract accounts?
Again, to recap, the **externally-owned accounts (EOA)**, they have a **nonce** and **balance**, they don't have a **storage** and **code**, but it's self-explanatory and the **smart contract accounts (SCA)**, they have both **storage hashes** (see picture) and **code**,
so basically inside the smart contract you can write code and deploy it, and based on this code you can do different functions,
depending on what you have included in the smart contract. So, yeah, this is more or less the thing.
And just keep in mind **this EOA is your Metamask wallet** or whatever wallet you use from a provider, and **this SCA is just a smart contract** which you deploy on Ethereum.

## slide: Ethereum accounts: EOAs

Again, the difference here is that with this **externally-owned account**, we **have a private key**, unlike smart contracts. **Smart contracts**, they **don't have any private key**.
Here (for EOAs) you have a **private key**, and with this private key, basically this is a key pair, **public-private key pair**, it's called, where you can **sign transactions** from your wallet and take the key, for example, interact with smart contracts, send money or whatever you do on blockchain. Everything is derived from this wallet.

And usually it's confusing, but I'm not sure whether you spoke about this with Prof Prinz, but **private key is not the same like seed phrase**.

---

### "seed phrase in crypto"

A **seed phrase** (also called a *recovery phrase* or *mnemonic phrase*) in crypto is a **list of 12–24 simple words** that acts as the **master key to your wallet**.

---

#### 🔑 What it does

* It is used to **generate all your private keys**
* It allows you to **restore your wallet** on any device
* Whoever has it can **fully control your funds**

👉 Example (DO NOT use this — just illustration):

> `apple river chair sunset ...` (12–24 words)

---

#### 🧠 How it works (simple idea)

Seed phrases follow a standard like BIP-39:

* Words come from a fixed list (2048 words)
* Each phrase encodes a large random number
* That number → generates your private keys → controls your crypto

---

#### ⚠️ Why it’s extremely important

* No password reset exists in crypto
* If you **lose your seed phrase → funds are gone forever**
* If someone **steals it → they can drain your wallet instantly**

---

#### 🔐 Best practices

* Write it down **offline** (paper or metal backup)
* Never store it in:

  * screenshots
  * email
  * cloud storage
* Never share it with anyone (no legit service will ask for it)

---

#### 🪙 Where you see it

When creating a wallet (e.g. MetaMask or Trust Wallet), you’ll be shown the seed phrase once and asked to confirm it.

---

#### 💡 Key takeaway

A seed phrase = **your crypto wallet itself**
Not just access — **ownership**

---

If you want, I can explain how a seed phrase actually turns into private keys step-by-step (it’s surprisingly elegant).

---

So usually people get confused, okay, I have a wallet, if I export my private key, then I have the same wallet on my other device.
Yeah, in theory it's something similar, but your wallet like Metamask or Coinbase wallet or whatever wallet you use, they have this **seed phrase** which is like a **master password** for all your accounts which are stored underneath your wallet.
So basically **in one wallet I can have multiple accounts**, so account 1, 2, 3, 4, I can have unlimited.
So if I **export only the private key**, I will **just export my account**, but I'm **not exporting my wallet**.
And if I want to **export the wallet**, basically I need to **take my seed phrase** and put it in some other wallet provider. Some other wallet.
Yes?

student: But if I export my private key,...

Ivan: ... you will export to a new account.

student: Yeah, but the new wallet then should be able to determine my balance, right?

Ivan: If you need this account, yeah, but if you have ten accounts in your wallet and you export only the private key of account 1 to the new wallet, then you have only that account 1.
But if you export, I mean, you take the seed phrase, you export all the accounts together.
So it's an easier transfer.

Between externally-owned accounts, I mean between users (or wallets), you can only send money, in that case, Ethereum.
And with the externally-owned accounts you can interact with smart contracts.
This is very important. Later you will see one smart contract can call another smart contract.
But the first caller smart contract should always be in code from externally-owned account.
Basically only this wallet can initiate a transaction. Only a person, the smart contract itself, cannot start any transaction.
Unless it has,... I mean, if you create something like a script (or bot), implement your private key and then you see this private key controls my wallet, you can call those smart contracts, yes, but in that case, you automate this process.
But a smart contract cannot call another smart contract without externally-owned account.
Okay.

## slide: Ethereum accounts: Smart Contract Accounts

So, now we come to the topic of **smart contract**.
Again, every smart contract has an **identifier address**.
And this identifier address has been **derived out of the deployment** of the smart contract.
So whenever we create a smart contract, and we deploy it on chain, we create a hash and transaction,
and usually this smart contract address is derived out of this transaction.
Similarly, like the externally-owned account, so in that case we derive it from the transaction.

When you create a smart contract on Ethereum, you always pay some **gas fees**.
Because this is, remember, you are changing the state of the blockchain,
and every time you **change the state** of the blockchain, you need to pay gas.
So usually **the more complicated the smart contract is, the more gas fees you need to pay**,
so you need to get prepared to pay this money on your wallet.

And yeah, this is the last one, **cannot initiate transactions**,
meaning that every time you call a function on a smart contract,
it is being executed initially from externally-owned account, this normal wallet, Metamask.
Just keep in mind, **externally-owned account is the simplest wallet you should have on your device**.

## slide: Smart Contracts

And yeah, **what is a smart contract?**
Like the idea is actually old, it's from 1990s,
and there is this guy, **Nick Szabo**, he **created the concept behind a smart contract**,
and the idea is that it's not smart, it's not a contract,
it's basically a simple program which you can create in every programming language,
but in that case the difference is that this program is **immutable**, it's deployed on chain,
and you are sure that whatever you write in the logic of the smart contract will be executed on chain.
And no, but... the idea is **no one can change it later once that contract is deployed**.

## slide: Smart Contracts: Vending Machine

The **example** which we usually give is with the **vending machine**,
so we know how the vending machine works.
If we want to buy a snack or a drink, we just insert a coin,
then we need to choose the code of the item we want to buy,
and if we put more money then we are sure that we will get a change back.
If we put less money then the vending machine will let us take our drink or snack,
and if there is no drink or snack inside the machine under this code, we will get our money back.

So, think about this example **also with the smart contract**.
Everything is predefined, everything is secured, whatever you do there you are sure that you will execute it eventually,
unless there is some bug or issue which no one knows,
and I can also explain later if you want to talk about some blockchain hacks,
but pretty much if you have audited the code and if you know what it is doing,
you should be sure that 100% it will happen.

## slide: Smart Contracts: Escrow Contract

This is a project, I mean I simplified this example of the smart contract,
it is called an **ESCROW smart contract**, it was also a project we did with Fraunhofer,
like three years ago I guess, it's about **selling electricity**.

And usually, like we do on blockchain, here on that example you have **two parties**,
Alice and Bob, Alice is selling electricity. Alice wants to buy electricity,
and Bob is selling electricity, but here we don't have a trust,...
**we don't want to put trust between Alice and Bob**,
so we decided, okay, why don't we create a smart contract,
and we can enable basically electricity trading between different people
without going to each other based on smart contracts.

And how does it work? Whenever for example Alice who is the buyer of the electricity,
when she **deposits money inside the smart contract**,
the other guy, in our case Bob, can generate electricity, send it to her house,
and if the houses receive - I mean there should be some **sensors (oracles)** to determine that electricity was received - the **smart contract will release the funds (to Bob)**.

So, this is just a trivial, simple example of what you can do with a smart contract,
and it's called ESCROW smart contract, I mean it's pretty famous, they use it...

student: Okay, so, if the internet goes down, would this go down?

Ivan: Yeah, I mean they should be somehow connected to the blockchain,
if you don't have internet, if you don't have a way to send the transaction.
So yeah, in case you don't have internet connection, that won't work.

student: I mean it is hard to imagine where exactly the code is run,
does it run on every node on the blockchain or?

Ivan: I will show you, no it's not run on every node, it's just a smart contract,
you deploy it, but this version of the code has access to every node in blockchain,
I mean not even every node, you also have access to the blockchain
because it's transparent, I mean I will show you later,
you have a block explorer, you see everything if you deploy it,
if you have a wallet you can directly see it. So, I mean I can deploy it and you together do it,
but even I don't need to know...

## slide: Smart Contracts in Ethereum (1)

So **how do we write a smart contract?**
Typically it's written in **Solidity**, I mean **on Ethereum**,
we are talking about Ethereum or **EVM chains**, which are Ethereum virtual machines chains,
typically like not only on Solidity, there are other languages like Viper and some new ones,
but let's say **80-90% of the smart contracts are written in Solidity**.

What else do we have here?
The smart contracts, as I said, are executed only,... the functions when you call the smart contracts are **executed only from the externally-owned accounts**.

We can have multiple,... in one transaction we can call multiple smart contracts together,
I mean, they can be **nested** (informally: "chained") or just **individual** smart contracts,
we can call them in one transaction.

---

In Ethereum terminology:

* Contract calls inside other contract calls are typically described as **nested calls**
* Example:

  * EOA → Contract A
  * Contract A → Contract B
  * Contract B → Contract C

➡️ These are **nested within the same transaction execution**

---

🔑 Key distinction

* **Nested** → one call happens *inside* another
* **Chained** → more informal description of the sequence

For lectures and exams, **“nested”** is usually the more precise term.

---

A really important feature which you also should remember is that all the transactions are **atomic**,
meaning that either they will succeed or they will be completely reverted.
So I will call a smart contract and I need to be sure 100% of whatever function I call in the smart contract will succeed.
If it doesn't succeed, then the whole transaction is reverted and it's not recorded on chain.
Basically it's like I didn't call the smart contract at all.

There are some...
First of all, it's **immutable**, once again, whatever we deploy once, it stays forever,
so that's why it's important to be sure that if it's a project which is going to be used from many users,
and it's a final version, there is some audit, so we are sure that it's working,
otherwise if there is some bug, then we cannot change it anymore.

There is an option where you can **delete the smart contract**,
but it's **not actually deleting**, you are **deleting the storage of the smart contract**,
meaning that all the information which has been stored,
but I think since last year Ethereum removed this opcode, it's called the opcode `SELFDESTRUCT`.
Later I will explain what is an opcode, but I just wanted to keep that because in the initial version of Ethereum this was also an option to delete the smart contract, but again, it's not only deleting, all the interactions,
I mean **all the transactions which have been in the past executed from this smart contract will still be visible on chain**.
So you basically **only delete the code of the smart contract**,
but you **cannot create new transactions later on** once you execute this `SELFDESTRUCT`.
Any questions?

Yeah.

student: Is it possible to have a **smart contract with limited time**,
so after a specific date it doesn't work anymore.

Ivan: Yeah.
Yeah.
Yes?

student2: So how would you do the code deletion if it would be stored on the blockchain?

Ivan: As I said, I mean you don't delete exactly the smart contract,
you more or less **disable** it from being used later on.
So everything, again, all the transactions which have been executed with this smart contract,
everything which is being done in the past with it,
it's something like a switch, you are not allowed to execute or do the same transactions anymore in the future,
that's why they call it `SELFDESTRUCT`, basically, it stops,... **it prevents from being used again**.

---

🧠 Key concept behind this

* Smart contracts are **immutable**
* You **cannot truly delete history or code from the blockchain**
* `SELFDESTRUCT`:

  * disables future interaction
  * may remove code/state from the current state
  * but **past transactions remain forever**

---

⚠️ Important modern note

In newer Ethereum updates:

* `SELFDESTRUCT` behavior has been **restricted/deprecated**
* It no longer fully “deletes” contracts in the old sense

---

🔑 Takeaway

You don’t *delete* a smart contract — you **deactivate it going forward**, while history stays permanently on-chain.

---

## slide: Smart Contracts in Ethereum (2)

So here, I mean it's a little bit complicated,
but you will see it later on when you deploy the smart contract,
that's why I wanted to give you a brief introduction on what you should expect.
Like a smart contract, which you see as a smart contract,
is just **written in Solidity**, which is, again, a **human-readable language**,
but **on a low level** what is happening,
we have a **bytecode** which is **executed from the Ethereum virtual machine**.
I mean **we use Solidity to create the logic and everything**,
but **then this is compiled with a compiler to bytecode**,
and on Ethereum, like **on the blockchain you only see the bytecode**.

There is also another terminology which is called **ABI**,
and this is basically **a JSON file**,
which is **showing all the functions and methods which you have on your smart contracts**,
and this is like an instruction how you can connect your front-end to the smart contract.
For example, if I have a website,
and I click a button, I need to call a certain function of the smart contract,
this is predefined in this JSON file, and this JSON file is called ABI.
You will see it later on, I mean it's not very important to remember it,
but if you see it later, just remember ABI,
this is the **bridge between your front-end and the smart contract**.

student3: I saw something about like when you call some function on a smart contract,
you're actually using the first or last four bytes of the hash of the part of the ABI.
Is that true?

Ivan: The ABI is more like a human-readable way so that you see the functions,
but in reality if you call the first or last bytes, you call the bytecode,
but you see it later, I will show you later on the other slide.

Whenever we deploy it, again, there is a **transaction fee** which needs to be paid from your wallet,
in our case, again, **from our externally-owned account**.

The **smart contract**, again, **has its own balance**, if you remember the initial image,
which I showed you here, with the smart contract and the Ethereum wallet.
We have now a balance,
that means **we can send and receive money from the smart contract**.

We **can call the functions from the smart contract**,
but again, one smart contract can call another smart contract,
but **the initial transaction needs to be executed from your wallet, not from the smart contract itself**.

## slide: Smart Contracts: Example

And yeah, now we come to a small **example**,
because I'm talking, talking, but this is how it looks like.

This is just a **simple storage**, I name it like a `SimpleStorage`,
where you have two functions, a `set()` function and a `get()` function.

Every smart contract has a **name**, in our case it is called `SimpleStorage`.

You also have a **version of the compiler** which we will see later,
but usually it always starts with this `pragma solidity` and the version of the compiler which you are going to use,
we have different versions of it.
You will see different numbers here.

But the idea here is we have a small smart contract
which is a storing unsigned integer, called `Num`.
And here we have basically set the number,
like `Num = 5;`, it will be `5`,
and then if you call this function, what's the value of `Num`,
we will get `5`. Pretty much straightforward.

You have unsigned integers and normal integers,
basically signed integers, where it's a negative and positive number,
so you should keep that in mind what kind of numbers you want to use.
And you also should keep in mind what kind of Solidity versions
you use to compile a smart contract,
because some Solidity versions have bugs,
and usually the stable versions are after `8.0`, I will say, or nine and above.

## slide: Smart Contracts: Turing-completeness

Okay, again going back to the features of a smart contract,
I mentioned it last time, but all the smart contracts are **Turing-complete**,
meaning that if we have enough time, we are sure that
eventually we will execute the smart contract, however,
if you think more about testing, the more execution you have,
the more gas fees you need to pay, therefore, it's **not infinite**,
it will be **eventually limited by the transaction fees**,
and this is why **we don't have infinite loops** here.
I mean, infinite loops you cannot get because eventually you run out of gas
and the program will be stopped.
I mean, stopped means the **transactions will be reverted**, since they are **atomic**.

student3: How long would the longest be that it could run?

Ivan: I think it's based on the...
Every block has a maximum size limit,
so basically the amount of gas you can pay,
so if your transaction will not fit inside the block, it will be eventually reverted.

student3: And if a transaction is reverted, do you get back the gas fee or is it lost?

Ivan: No, it's lost.

student3: It's still lost.

Ivan: Yeah, so basically you lose the money.

## slide: Smart Contracts: Opcodes

Okay.
So, now that you saw what the smart contract is,
and I explain you that this smart contract in Solidity is compiled to bytecode,
you...
I just generated this image to basically give you a **visual explanation**
what's a **bytecode** and what's an **opcode**.
Because on a lower level, I mean, when you write a smart contract in Solidity,
you write it by functions and variables and everything,
but on a compiler level, we execute these bytecodes.
And these bytecodes are basically a version of the opcodes,
and the opcodes are basically those operations where we need to pay gas fees.
So, it might be a little bit confusing,
but we pay...
Our payment for executing or for deploying a smart contract is based on how many opcodes we use,
and **those opcodes are later on converted to the bytecode itself**.

So, here this is some opcode, like `PUSH`.
`PUSH` is basically pushing to the stack,
and `ADD`, for example, adding one number to another number,
I think on the next slide I'll give you an example.
But you should remember here that the opcodes are basically the codes which are paid,
and based on the opcodes, you can calculate how much gas fees you need to pay
for deploying a smart contract or for calling a certain function on the smart contract.

And, yeah, I found out that there are more than 140 different opcodes,
so basically depending on what kind of operation you do on chain,
depending on that you can calculate how much money you need to pay again.

student: We think about this example you showed in the previous slide.
If we store a value, like, imagine it as basically like an API that I can call of a server,
like, is that a model that makes sense?
Like, somebody calls it and says, okay, I want to store the number 42,
and then somebody else calls the getter function and says, okay, I now want to receive the number that's stored,
and he gets 42? And if so, where is this number actually stored?

Ivan: On blockchain.

student: It's a blockchain-based server, not on the machine-based server, but actually...

Ivan: Yeah, so basically you execute it on the node,
on this Ethereum virtual machine node, which is running Ethereum virtual machine software,
or operating system in our case.
But every time when you call, for example, store number 5, I think, it's on the next slide... Yeah, here is a good explanation.

## slide: Smart Contracts: From Function to Bytecode (1)

There is a function called `add()`, so basically here in that case, I'm summing `a + b`, right?
And this is Solidity, this is now the opcodes.
Opcodes is `PUSH1`, `ADD`, `MSTORE`, and `RETURN`.
And when you're on a computer, you have like, this is the hard drive,
you have also RAM, here you have also a memory, storage,
and the stack. Stack is basically the lowest...

- Okay, let's start first with the **storage**. So whatever you store on the storage, you store it on the blockchain, so basically you have access later on on the blockchain, I can retrieve it from the block explorers.
- The **memory** is basically only storing information inside the function, so when this function is completed, then memory is deleted.
- And **stack** is basically a small operation, when I just store... if my function is `a + b`, and then I say, the result of `a + b` should be summed with `c`, the result of `a + b` is stored on the stack. So this is a space where you store information, but for a really short amount of time, within the function in the memory. This is memory of the function.

student: So if we have an example of the number storage, and people like...
if I call it, I set it to 42, set it to 0, set it to 42, all the time,
then all these different states will be stored on blockchain forever.

Ivan: Yeah, so basically what we'll do, we'll call this function just normally,
this function will be executing these opcodes,
so basically take the first place, `a`, and the second place, `b`, then add them together,
then store this information.
This is just storing it for the function, for the lifetime of the function,
and then return it, but if we want to store it for the lifetime of the smart contract,
we just need to have `SSTORE`, which stands for storage,
and then we'll store it on the state of the blockchain,
which is basically here, think about it, just a big number,
think about the smart contract, it's a big number,
and whenever you change something, you change also this number.
This number is basically a transaction hash.
So if you want to retrieve this number, you go to the transaction hash and say,
okay, what was stored on that smart contract with this transaction hash,
and this transaction hash gets some access to the storage.

I probably explained it super stupid and you don't get it,
but it's confusing, it's not so simple,
but the thing is that you hear this, if you go to this Merkle tree from last time,
where basically all the transactions are stacked in the tree structure or something like that,
all those transactions are part of this tree,
and all those transactions are basically changing the state of the tree
and, hence, changing the state of the blockchain,
and whenever I need to retrieve information from the blockchain,
I just go and ask, okay, this smart contract, it gets a storage,
I told you that, there is a storage, just tell me what's the number stored on the storage source,
let's say one or two or whatever storage source it is,
and then it will retrieve directly the number which I stored on that list.
But this number is also correlated with the state of the Merkle tree change early.

student: The only thing that's confusing me is that if we have an actual application running there,
where every state is basically stored in the blockchain,
doesn't that generate an incredible amount of data?

Ivan: Yeah, that's why it's constantly increasing the size of the blockchain.
Every time when you save some information, you basically save some more information on the blockchain,
and you make it bigger.
That's why last time when Professor Prince was talking about retrieving all the blocks,
he said that the later you go with the blocks, the slower it becomes to retrieve the information,
because there are more transactions, more interactions, so everything becomes slower and slower.
But think about it just as an SSD card which is shared among all of us,
and whenever we send information to this hard drive,
basically everyone has access to this hard drive, everyone can save information there,
and everyone can retrieve information.
And this hard drive is not one hard drive,
it's just a copy of this hard drive that's saved from all nodes running on the blockchain.
So, whenever I change something from my node, from my hard drive,
then I basically say, okay, I changed the state of the blockchain,
please synchronize all your other nodes around the world,
that we have the same state, the same information which I saved from this smart contract on your hard drive.

student: Thank you.

Ivan: It's confusing, I agree, probably I also didn't get it from the first time,
but the longer you use it, the easier it will become.
That's why I would say this is not something you should understand from the beginning,
I don't think even the professor does...,
don't take this for relevant,
I just wanted to explain you how it works,
so that you have a little more knowledge about how it works.

## slide: Smart Contracts: From Function to Bytecode (2)

And yeah, as I explained on the previous slide,
those opcodes, there are different types of opcodes, more than 140,
and those opcodes, they have gas fees,
so basically depending on what we do on the blockchain,
we call different opcodes,
and these opcodes we need to pay for example, for `PUSH` we pay three units of gas,
and for `POP`, for adding, multiplying, saving on memory,
saving on the storage, jumping, calling data, like we have different pricing, tax, how much we need to pay for the opcodes.
And yeah, pretty much that's the,... on a lower level, on a compiler level.

## slide: Smart Contracts: Solidity introduction

So now going again to the smart contracts.
As I said, it's written in Solidity,
it can be also the second most famous language is **Vyper**.
Both of them are just **object-oriented languages**,
like Java and JavaScript, I would say.

The **Solidity compiler** is called **SOC**,
so basically it comes with a Soldity compiler,
this is the compiler which is compiling the bytecode,... that's compiling the Solidity to bytecode,
and then we have,... what do we have?... Let me see...
Yeah. Every time when you call it is a transaction.
This compiler is basically running on the Ethereum virtual machine,
which is the OS of the nodes.
Any questions?

student: Yeah, do these prices, these gas prices change?

Ivan: No, they are just fixed,

student: But the guys from Ethereum,
think about it, can they change it?

Ivan: Yeah, they changed the...
of course, updates of the EVM and updates of the Solidity also,
so eventually at some time, probably they can change it.
I actually never checked whether they have been changed since the beginning,
but eventually they can reach the consensus
where they make, for example, storing more expensive.

student3: But the gas fee rate does change all the time, as you said.

Ivan: Gas fees' rate changes every minute,
depending on the usage of Ethereum,
so if there are lots of people right now using it,
there will be lots of calls and it will be more expensive,
but this is probably,... if it's changed,
it's probably changed on some few years' time or something.

student: Are these gas prices for deploying the contract or using the cotract?

Ivan: For deploying and basically for interacting with the smart contract,
if you look at this, this is a function,
this function is basically calling those opcodes,
so basically it's calling `PUSH`, it's calling `ADD`,
and it's calling `MSTORE`.
And those opcodes, they have gas fees,
so whenever I call this function, I know how much gas fees it will take.

student: When I deploy it, there is another fee, right?

Ivan: Yeah, deploying the smart contract,
you pay only for the opcodes used for the deployment.
But if you use the... after you deploy the smart contract,
it's already existing, so you have now those functions of the smart contract.
And then, depending on the functions,
you can go to the bytecode and calculate it.
I mean, the wallet itself calculates it automatically,
but if you want to go one step deeper,
you can also just look into the opcode
and say, okay, the opcode costs three units of gas,
do I pay three or do I pay more?

student: So the example with the edge is doing the five opcodes,
that's for executing it,
and if I want to execute the smart contract,
I would pay the gas fee for those five opcodes.
And for deploying it would probably be another set of opcodes.

Ivan: Deploying basically means you create a smart contract,
your compiler will generate the opcodes for deployment of the smart contract,
and there will be probably some other.
This is only for the execution, this is only when I call the function.
But for the deployment you pay something else,
you don't pay this (on the slide) for deployment.

student: So the gas fees work like, for example,
storing costs like 20,000 I think,
so it's 20,000 times `c`,
and the `c` is that when the gas price changes, right?

Ivan: what is `c`?

student: Like any constant?

Ivan: Ah, the constant, yeah, exactly.

student2: So the cost of deploying such a smart contract
depends on the constant of the smart contract.

Ivan: Exactly, so the longer... the bigger the smart contract is,
the more operations I get,
and that means the logic will be more complicated,
and you need to pay more.

student3: Does it just depend on the size of the smart contract,
or also on what it contains?
When you deploy it?

Ivan: Good question. To be honest, I never looked that deep.
I would assume both, but I'm not sure.
I'm not sure.
I need to divulge it.
Definitely the size, because the bigger the smart contract is,
the bigger the state change, and the storage, everything.
So this is for sure paid, but what else I'm not sure.

student4: Can multiple people deploy the same smart contract?

Ivan: Yeah, you can deploy it, but it's not in the same block,
it will be in different blocks, or, yeah, in the same block, but different transactions.

student4: So who pays the gas fee if all the people deployed it?

Ivan: If you deploy your own version, it will be the same version,
but you pay from your wallet.
I can copy the same smart contract, which you deployed, and I can deploy it from my wallet.
So eventually, you will have the same logic smart contract
with different addresses,
but I will pay for my own, and you pay for your own version.
So I just upload an image, for example.
You upload the image from your computer,
I upload it from my computer.
There are two instances of this image,
living on the internet, but one is uploaded from you,
one is uploaded from me, so the one I uploaded,
I pay for it, and the one you uploaded, you pay for it.

student3: The same constant code with two different contracts.

Ivan: Yeah, but they have two different addresses.
So if I deploy the same smart contract that you deploy, we will eventually get two different smart contract addresses.

student5: So if the smart contract involves
me paying monthly someone else.
So the person who pays the gas fee,
is the person who deployed the contract.

Ivan: Yeah, but if you remember again,
you cannot pay the gas fees automatically.
You need to,... every time when you pay your monthly fees,
you need to call the smart contract,
hey, please pay to my landlord,
my apartment fee, yeah, my rent.
In that case, your wallet is calling the smart contract,
which is having a function, `pay_renter()`,
`pay_your_landlord()` or whatever,
and then you pay for the execution of these transactions
from your wallet, and if the smart contract has balance,
probably it will send the money to the landlord.
But in that case, I don't,...
if I call your smart contract,
I would pay your fees, I will pay your rent
using my wallet fees.
So think about, we have the same version of the smart contract,
every month you call a function of that smart contract
to pay your landlord, and every time you call this function,
you pay a small transaction fee.
And if I call this function on your smart contract,
I will pay the transaction fees,
but I will also pay your rent from the smart contract.
If the smart contract has a balance,
you're not to pay for the rent,
because if the balance is zero,
then the execution will revert,
because, I told you, it is atomic,
if there is not enough money in the smart contract,
it will just revert,
I would eventually pay the fees,
but the rent won't be paid,
because there is no money in it.

student: (question)

Ivan: (answer)

## slide: Smart Contracts: Solidity introduction (1)

So again if we go back to the example,
we have the `pragma solidity`,
which I said is a **compiler version**.
And we have the **name** of the smart contract,
and then we can have different **types**, like, as I said,
we can have unsigned integers (`uint`), signed integers (`int`), we can have boolean (`bool`), `string`, `address`.
Addresses are especially interesting.
So I said everyone on blockchain has an **address**,
so we need a **type of `address`** to determine
where the transactions are coming from.
We also have bytes, and there are different types.
I would say this is just informative,
so you know what to expect,
but depending on what smart contract you want to create,
you can look into different types.

And very important, there is **no timestamp (type)** on the smart contract.
**We use** this, I think it's called an **epoch**, **Unix timestamp**, something like this,
which is a big number from 1970s, which is representing our current time, date and seconds.

## slide: Smart Contracts: Solidity introduction (2)

What else do we have?
So it's very important that we have different variables,
so we have
- **contract-wide** variables,
- **function-local** variables, and
- **blockchain-global** variables.

And most interesting are the **blockchain-global** variables,
which means that those variables are **publicly accessible**.
So usually the person who send the transaction
to the smart contract is called `msg.sender`.
Everyone has access to this.
We can see the value, if we deposit, for example,
the example from before with the rent.
If we send money to the smart contract,
we,... there is a `msg.value`,
basically writing the value of the money
which we send to the smart contract,
and it's stored there.
And we also have `block.number`,
because every transaction eventually will be saved
on some block, so we can use the block number
to reference where the transaction was sent.
And there is also `block.timestamp`,
so basically, again, in this unique strip of time,
we can determine what was the time and date
of calling the smart contract.
We also have just simple operation like arithmetic,
plus, minus, whatever,
and comparison, logical and assignments,
just like normal.

student: How is it made sure that the blockchain global variables
are unique in the namespace?
Is there a transaction ID in front of the `msg.sender`?

Ivan: So `msg.value` can be same.
If I send you one, you always have the same value.
`block.number` is always unique,
because `block.number` basically is
~~the end of the transaction, as you always prefer.~~

student: But if I want to access the `block.number`,...

Ivan: From the transaction, you can see there is on the block explorer,
you can see the...
Those block explorer is the website where everything gets visually represented.

student: I mean, that's publicly available, I get it,
but if I want to, for example,
access them from somewhere they need to have a unique name, no?

Ivan: Yeah, it's `block.number`.
So if you write a smart contract and you say,
I want the `block.number` to be bigger than that,
in that case this function can be called only
from `block.number` which is in the future, in the near future.
This is the example where you say,
can we limit it, for example,
to not do the smart contract after this lecture, for example.
We can say whoever sends a transaction
to our smart contract,
and this transaction lands in a `block.number`
that is later than we specified,
when it's the deadline of this lecture,
then it will be reverted every time someone tries
to send the transaction to the smart contract
after the lecture, it will be reverted.

student: And I can, from the site, the smart contract, for example,
I can access `msg.sender`,
and you will always only need this `msg.sender`
from the guy who called it.

Ivan: Yes.

student: Okay, because there are probably thousands of messages
to send, right?

Ivan: Yeah.
That's why it's called **deterministic**.
The smart contract just compiles it without any errors,
and every time someone is calling it will just execute.

---

### deterministic

The missing word is:

👉 **deterministic**

---

#### ✔️ Filled version:

That's why it's called **deterministic**.
The smart contract just compiles it without any errors,
and every time someone is calling it will just do it.

---

#### 🧠 What “deterministic” means here

In Ethereum, **deterministic execution** means:

* Same input → **same output**
* Every node executes the **exact same logic**
* No randomness or ambiguity during execution

---

#### 🔗 Why this matters in your example

* `msg.sender` → always the caller of the function
* `block.number` → same value for all nodes in that block
* Function logic → behaves identically for everyone

➡️ So:

> If the condition fails once, it will fail **for everyone**

---

#### 🔑 Key takeaway

Smart contracts must be **deterministic** so that:

* All nodes agree on the result
* The blockchain stays **consistent**
* No conflicting states occur

---

If you want, I can give examples of what would *break* determinism (e.g., randomness, external APIs)—very important concept.

---

student3: So sorry, those values are then in the global namespace,
and they are per contract?

Ivan: Yeah, yeah, per contract.

## slide: Smart Contracts: Solidity introduction (3)

What else do we have?
We have **loops**, but they are not really a good option
to be used because the longer the loop is,
the more gas fees you need to pay.
So eventually, if you end up creating a smart contract
with a big list where you need to loop
over the items in the list,
you are basically saying,
hey guys, whoever calls my contract
needs to pay a bunch of gas fees.
So people would not be interested in using the smart contract.
That's why we use **mapping**,
on the next slide.

We use **arrays**, I mean here is the **declaration** of an array,
how you can **initialize** it,
then **adding elements** and **accessing elements**,
it's just trivial stuff, yeah.

student3: What's the difference between the two initializations of arrays?

Ivan: Just different type of initialization,
I mean both of them do the same thing,
just different syntax.
You can choose.
I usually use the first one, but you can use the second one.

student3: So then it doesn't change where it's stored.

Ivan: Yeah, just different syntax.

## slide: Smart Contracts: Solidity introduction (4)

What else do we have?
We have the **enumerations**, yeah, I'm pretty sure you know them.
We have **structures**,
so basically like a `struct`,
and each `struct` can also be initialized.
In our case, we create a `struct` called `student`,
this student `struct` has a `name`, `semester`, and `grade`,
so we can basically have those in an instance of the `struct`.

And then we have this **mapping**,
which I told you it's very popular in the smart contracts
because loops they cost lots of money,
so we usually have a `mapping`.
The `mapping` is between address,
usually between address and something else.
For example, if I'm the owner of a smart contract,
my address will read to, for example,
the owner balance will be 10 ETH, so it will read to 10.
If you are just user, maybe you put your address
and you have 5 ETH.
So this is a way how you map your address
to some integer or string or boolean or whatever.
You can have a **nested mapping**, which is a mapping inside a mapping, but it's a little bit more complicated.
But this is usually when you create a list of elements
and you say I want this student to get five Euros,
you don't go through the **loop** of this list,
you create a `mapping` so that you spend less gas fees and
repeat the elements from the smart contract.

---

### Loop

The missing word is:

👉 **loop**

---

#### ✔️ Filled version:

…you create a list of elements
and you say I want this student to get five Euros,
you don't go through the **loop** of this list,
you create a `mapping` so that you spend less gas fees and repeat the elements from the smart contract.

---

#### 🧠 What he means here

Instead of:

* Iterating through a list (**loop**) ❌ expensive

You use:

* Direct lookup via `mapping` ✅ efficient

---

#### ⚡ Why mappings are better

* **O(1) access** (constant time)
* No iteration needed
* Much lower gas cost

---

#### 🔑 Simple comparison

##### ❌ Loop approach

* Go through entire list
* Cost grows with size

##### ✅ Mapping approach

* Direct access:

  ```solidity
  balances[address] = 5;
  ```
* No iteration needed

---

#### 🧠 Key takeaway

Mappings replace loops when you need:

* Fast lookups
* Low gas costs
* Scalable smart contracts

---

If you want, I can show a real Solidity example comparing both approaches side-by-side.

---

## slide: Smart Contracts: Solidity introduction (5)

Okay, so what's that?
Yeah, so now we go to the **functions**.
The functions, they have different **properties**.

Some of the functions are called **view (functions)**,
which means that if you call this function,
you retrieve, you read values from the blockchain.

Some of the functions are called **pure (functions)**,
means that they are not even reading from the blockchain,
they are very rarely used, I would say.

And if this **function doesn't have a property** like view or pure, it means that usually this function
can save information, so write and read from the blockchain.

Usually the **syntax** of the function looks like this.
So we have the `function`, then `name` it,
then we have a `parameter` or a simple parameter,
then we have this property, whether it's `view` or `pure`,
and does the function return something, maybe it doesn't return anything.

## slide: Smart Contracts: Function Visibility

And there is this `public` which is coming on the next slide.
It is basically saying who can call that function,
because not everyone, not all users
can call that function on chain.
This is actually, I would say,
**exam relevant**, because it's nice question,
to say what's the difference between `public` and `external`,
what's the difference between `external` and `private`.
It's pretty straightforward,
I mean even from the name of the function you can derive it.

`public` means that it's open for everyone,
so everyone can call this function, externally-owned accounts can call this one,
smart contracts can call this function.
It's publicly available and open for everyone.

Then we have `external` functions,
which is somehow like `public`,
except it cannot be called within the contract.
So the contract itself,
maybe there is a function called `add()`,
and there is another function called `subtract()`.
Those two functions cannot call each other
within the smart contract.

Then we have another function, which is called `internal`.
In that case, you restrict, like no-one from outside can call your function.
Only your inherited smart contracts can call them,
or the smart contract itself.

Then you have a `private`, which is the **most restrictive function**,
and in that case this function can only be called
from the smart contract itself,
even the derived smart contract.
Because like there is an inheritance,
like I can have a mother smart contract, a children smart contract, which is inheriting all the properties from the mother smart contract.
In that case, `internal` means that the children
can call the function from the mother,
and in `private`, it means that the children
can call only functions inside the children.

student: For the `external`, with the explicitly prefix
of `this` keyword, is this `this` like in Java, for example,
where you refer them to your own method?

Ivan: Yes, yes, exactly.
Actually, Solidity is very similar to Java.

student: But why does the `external` then exist,
if it can be called from the contract,
when you really the keyword this.

Ivan: I mean, this is again a specific situation,
I would say just ignore it.
Just if you get this question,
say `external` smart contract cannot be called from outside,
I mean, can be called only from outside,
not from the contract itself.
That would be sufficient.
So basically, it means everyone can call it,
except the smart contract itself,
or other smart contracts. I would say this is sufficient.

student: And so this means that with `external`,
it can only be called by its own contract
and externally-owned accounts,
but not foreign smart contracts?

Ivan: Not from own, only from external,
from wallets or from other smart contracts.
But not from,... because there is a function, for example,
do you have a smart contract with 10 functions,
one, two, three, four,...
probably this function needs to call this one,
and this needs to call this one.
and this needs to call that one.
In that case, you have function one is calling function two,
in that case, you cannot.
Because `external` means that function two
can be called only from outside.

student: But if I prefix the call with `this`?

Ivan: ...you cannot call it from here.
Don't think about this, it will become too confusing,
just think it can be called only from outside.
Don't worry.
It will be just more confusing.
Just think that `public`, `public` means everyone,
if everyone can do everything,
`private` means no-one can do,
only the smart contract.
And then think `external`, which is `external`,
only from outside the smart contract.
`internal` means probably it's internal,
but `internal` in a way that the smart contracts
derived out of this parent, mother example.

## slide: Smart Contracts: Solidity introduction (6)

Okay, so what else do we have here?
We have **events**, which is something like a **notification**.
We can create an event,
and every time an event is triggered,
we get a notification,
similar like a mobile phone and apps.
So every time someone creates a transaction
or everything, we can **subscribe** to an event
that **that shows** someone just paid
to the smart contract,
or someone just called function A from that smart contract.
So think about **event is just a notification**.
I think this is important.

---

The missing word is:

👉 **that shows**

---

✔️ Filled version:

…we can **subscribe** to an event
that **shows** someone just paid
to the smart contract,
or someone just called function A from that smart contract.

---

🧠 What this means

Events in Solidity are like **logs/notifications**:

* Smart contract emits an event
* Frontend or backend can **listen (subscribe)**
* When triggered → you get notified

---

🔔 Real-world analogy

Like:

* 📱 “You received a payment” notification
* 🛒 “Order confirmed” message

---

⚡ Example in Solidity

```solidity
event PaymentReceived(address from, uint amount);

function pay() public payable {
    emit PaymentReceived(msg.sender, msg.value);
}
```

---

🔑 Why events are important

* Enable **frontend updates**
* Used in apps with libraries like web3.js or ethers.js
* Cheaper than storing data on-chain

---

🧠 Key takeaway

Events = **communication layer from blockchain → outside world**

---

If you want, I can show how a frontend actually listens to an event step-by-step.

---

## slide: Smart Contracts: Solidity introduction (7, with code)

Then, we have here different types of structures
we can have inside the smart contract.
First we have a `constructor`,
which is an initializing function like in C for example.
This is a function which is called only once
when the smart contract is deployed.
And usually this function
is determining the `owner` of the smart contract.
So if I say in the constructor `owner = msg.sender;`, which is the global variable,
means that whenever I create the smart contract
and deploy it, I automatically assign myself
as `msg.sender` and the `owner` of the smart contract.
And then I can have different functions
where I can create a `modifier`,
meaning that only the `owner` of the smart contract
can call this function.
For example, this one (`destroy()`).
I have a special function called `destroy()`
and then I want to destroy my smart contract.
I don't know other people,
even though it's `public` and everyone can call it.
I just want to **restrict that access**
that only me plus an `owner`,
which is derived from  `modifier onlyOwner` ,
can call this function and destroy the smart contract.
Otherwise, everyone would be able to call it
and just destroy my smart contract.
But again, `selfdestruct()`, I think it's not valid anymore since last year.
So those are just examples how you can use `onlyOwner`
but also `selfdestruct()`.
Yeah.

student: What are the function names
like these functions are actually predefined, right?

Ivan: The function is called `destroy()`.
You can call it `destroy()`.
You can change it.
Ah, this is predefined.
The `modifier`, the red ink is predefined.
This is predefined and this is predefined.
This is like the initializing function.
If it's in red then it's part of the Solidity.
If it's blue, then okay,
in that case, it's confusing,
but yeah, you can think about here,
the naming of the function.

student2: This is try functions are symmetrically the same, right?

Ivan: Yeah, this here, we use this `requirement`.
where basically I require that the `msg.sender`,
I mean the global variable,
is equal to the `owner`.
Basically the `owner` is this `owner`
which we defined in the beginning
with the requirement of the...
But they will be the same.
Just here you have the requirement here.
And here you have the requirement inside the `modifier`.
Usually this is for saving for spacing
to create a `modifier`.
And instead of just writing every function required,
you just put the modifier name
and you know that this is already defined.

~~student2: This is the, I don't know, some point of length, underscore.~~

~~Ivan: Yeah.~~

~~student2: You need it because this is basically the case for everything. I will work. I will work.~~

~~Ivan: I always need this.~~

## org

Okay, by the way, do we have until 2:30 or 3:30?
Because I'm not sure.
I don't know how long is the lecture.

student: The room is booked till three,
but the usual lecture time is later.

Ivan: Okay, I'll try to go fast.
Because there's lots of stuff here, but I can do this.

## Use Cases for Smart Contracts

This was just an example of a smart contract.
So going again backwards.
Smart contracts are probably boring, unless you find a useful situation
where you create a smart contract.
Usually most of the smart contracts
are deployed for creating some kind of projects.
**On blockchain, 90% of the projects are related to financial stuff,**
for example, sending money, investing money, whatever related to money.

### Fake Product Detection

But you can also create smart contracts,
which are just,... I created for example,
a smart contract which is saving...
ok, there are different smart contracts,
but it was saving the,
you know, if you **buy for example, some expensive goods**,
they get this **unique identifier number**.
And usually, some people can take this number
and produce **fake**, for example,
**watches or bags** or whatever,
with this same identifier.
In that case, you can avoid this problem
because you can create a smart contract,
which every time there is a new fake produced
for a new bag or a new watch, this bag or watch has a unique identifier
and this identifier can be stored,
for example, inside the smart contract.
So whenever someone buys this bag or watch,
they assign this identifier to his wallet.
So if a second bag is produced
with the same product identifier,
in that case, the user who is buying the bag
can **go on-chain** because again,
everything is publicly available,
they can **go to the retailer official website and compare, okay, is there another item with this identifier already sold?**
If there is, in that case,
means that a person who is selling this bag or this watch
is basically malicious, he's fake,
because he's selling another bag with the same identifier.
Eventually, you can prevent fraud.
I mean, this is if you think about some off-chain use case.

### Fake Patient Information Leaflet (Packungsbeilage) Detection

We also created a version of a smart contract
where every time you open a medicine,
you have this paper with the instruction
how you take the medicine.
I mean, it is just paper, you need to read it.
We created a version where you can scan a QR code
and you basically take the whole instruction
of this medicine online and you can read it, for example,
from your phone or from your computer.
And now here comes the thing.
I can just store it on Google Drive,
I can store it on my server wherever I need.
But what happens if at some point there is a hack
and someone changed, for example, the instructions
to instead of take one pill, take five pills,
maybe dangerous for the person.
In that case, what did we do?
We hashed this version of the instruction,
we put it on chain and you see every time
when you open this PDF with the instructions,
compare the hash and if there is even a small change,
for example, one letter change,
then the instruction which you see
will be right to another hash
and you can see that it was basically changed.
And if this change is not from the pharmaceutical company,
then you should ask yourself,
should I really trust these instructions
from the medicine or not?

These are just like examples of how
you can create smart contracts
which are not related to financial stuff.

One last example I give you
and then we go to the financial stuff again.

### Checking Correct Wind Turbine Inspection by Insurers

There was a big project where people in Denmark
were producing those wind turbines
for creating electricity.
In those turbines, they happened to break very often,
especially when there is strong wind or some,
yeah, bad weather conditions.
So the insurance needs to pay for the repayment
of the broken thing.
But the insurance pays only if the guys who inspected
the turbine did their job correct.
And how did they do it?
They have a big, how do you call them?
Like these gadgets, it's in German, it's Drehmomentschlüssel,
it's like torque ..., like a big device which you just,...

student: torque wrench

Ivan: yeah, torque wrench, I think it is that.
So they need to basically make an inspection
with a certain power that every bolt
and every basically is done correctly,
which is every bolt is put correctly
and with certain amount of power.
So that in case there is a huge wind and everything,
if it breaks, they can always say,
well, but this guy who inspected it he did not do the proper inspection.
So this device is a booted device
which is connected with the blockchain
and it sends the data to the blockchain.
So if in the future this wind turbine breaks down,
we have the data from the inspector,
saved on chain and the insurance company
can go to the smart contract and state all the values
which were stored on chain and if everything was correctly,
they would just pay for the repairment
because they're sure that the repairment was done correctly.
This is an example of creating smart contract
outside the financial domain.

But usually also, as I said,
most of the smart contracts are inside the financial incentivation, like I want to make money off of this smart contract.
Yeah.

## slide: Smart Contracts: Solidity introduction (8, Reentrancy, Attack)

student3: What does the `msg.sender.call.value` do?

Ivan: Which one?

student3: On the left near the bottom `msg.sender.call.value`.
Just the last one.

Ivan: Yeah.
This one.
Yeah, this is basically, yeah, I will explain how it works.
So this was **one of the first hacks**
that happened **on Ethereum**. It is a smart contract
where people deployed a bunch of Ethereums
inside the smart contract and they were earning money.
And at some point there were lots of Ethereums,
I think even 10 percent of all Ethereums in circulation were in that smart contract
~~captured the guys from the other team who created the smart contract~~, it's really a nice project.
But **there was a small bug**,
and the bug is actually here.
Here's a problem where, think about, you have a smart contract
which is storing some Ethereum inside the `balance`
and then you have this function called `withdrawFunds()`.
And `withdrawFunds()` is basically asking,
okay, who is the sender or who is calling this function, `msg.sender`?
And this is `balance` which is a mapping.
Yeah, this is a mapping.
Thus he has more,...
because everyone who deposits his money has a `balance`.
So if I deposit money on Ethereum, my `balance` will be one Ethereum.
And think about this... this is in Wei, basically the smaller unit of Ethereum,
but think about this is one Ethereum ~~just before~~.
So if I call this function and I have previously
deposited one Ethereum to the smart contract,
and if I call `withdrawFunds()`,
usually what should happen?
The smart contract should check my `balance`.
So this guy has deposited one Ethereum.
Did he deposit it,...
Did he want to,... does he want to withdraw more money
than I'm allowed to withdraw?
For example, there is a hard cap per transaction,
I cannot withdraw more than that on Ethereum,
but I want one.
So in that case I satisfy this requirement.
Then there is another requirement:
Do I want to withdraw less than one Wei,
or more than one Wei, we can also ignore that.
And if I meet all those three requirements,
then the smart contract says okay, everything is fine.
Now just take the `sender` and send him the money
he wants to withdraw, basically the one Ethereum
which I have here.
And when I receive my money,
then my `balance` needs to be updated.
So basically now I don't have one Ethereum anymore,
I have zero.
Because my `balance` is one minus one, which is zero.
And then they store my last transaction.
So everything looks fine, right?

But there is a small problem because the transactions
on Ethereum they literally execute in the way
they are written here.
So there was one guy who said okay,
there is a problem, he identified this bug
couple months later and he created this attack which is really simple.
I mean just one contract for the attack.
There is basically a `constructor` that you don't need
to know this, but what is the attack actually doing?
It says okay, I want to withdraw my deposit.
Basically, first he creates a deposit
and then he wants to withdraw the deposit.
And then what is he doing?
He has again here to deposit one Ethereum.
And then he says now when I deposit my Ethereum,
I want to withdraw it.
Think about it, there is no withdrawal time,
everything is satisfied, so he immediately deposits
and after the deposit he wants to withdraw the same deal.
And usually, I told you, the smart contract can receive some money.
And in order to receive some money,
you need to have a function called `payable`
meaning that I can send money to the smart contract.
Otherwise I'm not able to send money to the smart contract
and transactions will revert.
So this guy just said okay, I will deposit one Ethereum
and then I will withdraw this deal.
But whenever I try to withdraw this Ethereum
and I reach this point here,
exactly the time which is sent,
the money will be sent, so think about one Ethereum
going here, the `function () payable`.
Then the `balance` of this smart contract becomes `+1`.
But this function has something else.
Like whenever it receives the money, it doesn't stop.
It just calls the same withdrawal function one more time.
So in that case we send the money, we go here,
**we send the money, but we don't reach to the update of the `balance`**.
So essentially what is this attack link?
It's just calling again the withdrawal function.
It checks again the requirement, it satisfies the requirement,
then sends another Ethereum.
Then does this like in a loop,
probably there are hundreds of Ethereum here,
then withdraw all the money,
and when the balance is basically,
I mean here, it doesn't have more Ethereum to withdraw,
it just updates the final status
and this guy ends up stealing 100 Ethereum
from the smart contract despite he initially only deposited one Ethereum.

And how was it fixed?
The fix was really simple,
you just need to **swap the lines**.
So basically you first need to update the balance
and then you need to send it.
I mean at first it doesn't look really problematic,
but, yeah, it happened in the one of the first and largest hacks/attacks.
And everything is fine, as I said,
you just need to swap the places of the functions.

student3: How much ETH was stolen?

Ivan: Just Google it. The **DAO Hack**. It happened 2017 or something like this.
Like it was **around 3.6 million ETH** (**about 10% of all ETH supply** at the time).

student: Is this some kind of concurrency between calls generally?

Ivan: Yeah.

student: Because I feel like, if you send,
if you call this `withdrawFunds` function,
100 times in the same second or in the same moment,
then also you would have the same problem, right?
Because you check it once in the beginning
and then in the end you end up...

Ivan: Yeah, this is actually the problem,
I would say more or less with Ethereum,
because in that case this compiler version,
I think right now the different compiler
is checking everything, not sequentially,
like one by one, but at that point
this compiler also doing the...
I think there is now by the code a prevention.
This is called **reentrancy attack**.
Google "reentrancy attack".

## slide: Smart Contracts: Solidity introduction (9, Other Attacks)

And there are other attacks which I would say
we will skip them because we don't have much time.
I will just briefly give you explanation.

### overflow attack

On the left side you see **overflow attack**,
which means that we have a Uint8 (`uint8`), for example,
which is storing numbers between 1 and 256.

---

From [Solidity docs](https://docs.soliditylang.org/en/latest/types.html#integers):

`int` / `uint`: Signed and unsigned integers of various sizes. Keywords `uint8` to `uint256` in steps of `8` (unsigned of `8` up to `256` bits) and `int8` to `int256`.

`uint` and `int` are aliases for `uint256` and `int256`, respectively.

---

You can overflow, if it surpasses 256
it will go again from 0.
So we can have this attack.

### front running attack

We can also have another attack where...
Here in that case,
we are looking for some word.
And this word is basically... the hash
is equal to the hash version of this word.
And if someone finds a word that satisfies this hash,
he publishes this word here,
he should get 100 ETH.
But in real world,
there are people who are basically **observing the mempool**.
And if someone finds that you found already a solution,
you will just create the same transaction,
**pay a little bit more gas fees**
and **queue a front running**.
So basically without any effort,
he will use your solution to get the money.
This is called **"front running attack"**.
You want to avoid this.

But I would not explain you how to avoid this
because it's, again, too complicated,
but there are also tricks how to avoid this.

So I would literally skip those stuff
because it's not really cool,
but I will show you like a brief example
of how you deploy a smart contract.
Because before we end I want to show you something else which is cool.

## Recommendations: Learn Smart Contract Programming, Remix

So if you want to **learn smart contract programming**,
I personally did it from this website
called [www.smartcontract.engineer](https://www.smartcontract.engineer/).
It's a nice website where they have different challenges.
This guy also has a bunch of YouTube videos,
so you can watch, where he explains everything.
Right now, the guy actually started also teaching Rust,
because Solidity is for Ethereum,
but if you want to deploy smart contracts,
for example, on other chains, like Solana,
usually you would use different languages.
Some of it is in Rust.
Yeah, you can use that as well.

Now that you know this website,
I just took an example,
and the example is this one here.
So you can deploy the smart contract using VSCode, for example.
But there is a really cool online compiler
which is called [Remix](https://remix.ethereum.org/).
I would say just write it down
because if professor asks you to do some assignment
it is easier to do it.
And here on the left side,
we have a compiler and we have a run and deploy.
So basically you need those two buttons.

Once you have a smart contract,
for example, I have a smart contract here
which is, if you check what this smart contract...
Yeah, so we have an `owner`.
Maybe I need to make it a little bit bigger.
So we have a smart contract where
I just say when I deploy my smart contract,
I want to be the `owner`.
So the guy who is deploying the smart contract
will be automatically set to the `owner`.
And then I want to have a function
where probably in the future I want to change the `owner`
of this smart contract.
But I want to be sure that the `owner` is not address 0.
Address 0 means that if you assign the `owner` to address 0,
no one has access because no one has access to a wallet called 0.
There is no private keys assigned to this.
And yeah, if I want to deploy that,
basically I have an option here on the left side
which I use, I can see different compiler versions.
Usually, as I said, if you use a compiler which is above 8, it's pretty safe.
I compile it.
And if there are no errors, here you see the green tick.
That means it's ready to be deployed.
Then I click here on the button below
which is "deploy and run transaction".
I deployed it once so I delete it.
I can choose the name of the smart contract
because think about here I can have
multiple smart contracts inside the file.
I choose to deploy the smart contract called allnobov.
And here on the environment, I can choose
whether I want to deploy it locally.
It means using Remix I don't need any money.
Or do I want to deploy it using my Metamask wallet?
So here on the browser extension, I have two wallets,
one is **Phantom for Solana**.
I think they also support Ethereum and Metamask.

---

Gemini: "metamask vs phantom wallet"

**MetaMask** and **Phantom** are both leading, non-custodial wallets designed for Web3, with **MetaMask specializing in Ethereum/EVM chains** and **Phantom in Solana**. Choose MetaMask for vast DeFi/DApp access on Ethereum/Polygon, or Phantom for superior user experience, built-in NFT management, and low-fee transactions on Solana, Ethereum, and Bitcoin.

---

So I said, okay, I want to deploy this with my Metamask.
So I will pay some fees,
but I want to see my smart contract on-chain.
If I don't, if I don't have money,
I'll just use the **Remix virtual machine**.
So it means that it will simulate a compilation
and deploy locally on Remix,
but you don't see it on-chain.
Let's say I deploy it now.
I click "deploy".
I create a new hash here.
Metamask just pops up and says "Hey, you have this smart contract. You need to pay network fee, then the maximum gas fees.", whatever, I agree.
And I pay whatever Metamask is calculating
for my smart contract.
And here now the smart contract is pending.
I can view it on Etherscan.
It will take some seconds.
So now it is pending, over here.
So once here, it stops showing pending,
I will eventually see this is my wallet
and I will see the smart contract address of the smart contract which I just deployed.
Okay, it was deployed. You see "success".
Now I have the smart contract which is blank.
You don't see anything.
But if I click here on the contract,
you'll see the **bytecode**.
And because you don't want to see the bytecode,
because you don't understand anything,
we just want to verify.
But the thing is only the guy who has the smart contract
can verify it because I know which compiler did I use.
In that case, I used Solidity compiler.
Then I need to check which version did I use for the compiler,
which is here, 8.31.
So I go and enter 8.31.
I hope it's this one or the other one.
Then I click continue, 3.
And I should...
Yeah, I should put the smart contract,... just copy and paste it.
Here,... and if it's verified,
it should be verified...
Think about it is working,
then you should see basically the same code here.
But you should see on the address of the smart contract, which is here.
So here you'll see the code.
This is the idea.
So think about this is, again, this is the website
where you can go and find smart contracts,
look into their code
and interact with the smart contract directly from the block explorer.
I mean, in order to interact, you need to connect your wallet
because again, for every transaction you need to pay fees.
But eventually, what you should remember is
that if you want to deploy a smart contract for the first time,
I would suggest using this one, a Remix.
You just copy a smart contract,
you decide whatever you want to do,
you compile it and then deploy.
I mean, it's literally one to two minutes work.
Pretty sure it would be easy to do.
Any questions on that part?

Because I know you were expecting something fancy.
How can I create a smart contract?
I mean, nowadays,
ChatGPT can create even better smart contracts than me.
I would say I spent three years only creating smart contracts
and now it's the level that ChatGPT is beating me sometimes
if you want to compete.
So I would say it's good to understand how Solidity works.
I would say that it's always nice to understand
what your smart contract is doing
because whenever you deploy something
or you use it in production, especially with money,
you need to be sure that your smart contract is not going to be hacked.

## Bug Bounties, code4arena, immunefi

And coming to the hacks,
I would say that we use the last five to ten minutes
for me showing you that
if you're interested in the blockchain stuff,
there are pretty cool places and websites you can explore.
One of them is [code4arena](https://code4rena.com/bounties),
which is I started this journey like probably two to three years ago.
There are a bunch of protocols or projects for companies
which are deploying their smart contracts
and those smart contracts, they need to be audited.
And those protocols of those companies,
they spend a lot of money for auditing smart contracts.
And I would say if you go to this Code for Arena website,
there are contests, you can audit smart contracts
and you earn really good amount of money.
And I would say six digits per year is realistic. I know two people who did this with less than six months of experience.
There is also a huge opportunity for...

student: How much money is realistic?

Ivan: Six, seven digits.
I know one that made six million in one year.
From this one and... and [immunefi](https://immunefi.com/bug-bounty/)...

So here you find protocols which are already deployed,
but you can still look into their smart contracts
and you see how big "MAX BOUNTY" is.
So, this one is paying one million for a person who finds...
Usually the bugs are critical, medium and low probability to happen.
So if you find critical bug,
they pay you one million, if you find medium bug, they probably pay you like two hundred thousand and low probability bug less.
But on this websites are already projects that are live,
there are thousands of projects.
Some people what they do, they just look into hacks
that happened in the past month.
And because most of the hacks, they can happen in multiple protocols simultaneously.
So if they identify that a hack happened on some protocol from here,
they just look for similar smart contracts
which are using similar functions or similar idea
and they just report the problem if it exists and they make money.
I mean, I had one student, I supervised a Bachelor's thesis,
he found a bug from here.
I think it was rewarded, I think, five thousand or something.
And he didn't really find the bug.
He created an article about this bug, and he said,
no, but I know another project who is doing exactly the same thing.
Let me see if their code is basically the same.
And it happened to be absolutely the same smart contract with some small differences, of course.
But he reported the bug, I mean I think two days later and he was rewarded.
So if you are good in Solidity, I would say you can look into this [immunefi](https://immunefi.com/bug-bounty/).
If you want to look into a smart contract which are not yet in production,
it's this [code4arena](https://code4rena.com/bounties) which is basically projects
for deploying their smart contracts before production.
And here in that case you are **competing with other people**.
Because on one contest, for example, here on "active projects",
there is currently this contest, here but it's Rust, it's not Solidity,
you see there is currently Rust, but the past, probably ... Solidity,
so there are probably like a thousand or two thousand people here competing
and eventually they will give $56,000 to all the people who reported some bugs.
But here you are sure that there are bugs.
Those people are always spent.
It's the matter of how much share from this pool you get to win.
The more bugs you find, the more money you get rewarded.

student: Do you get paid in ETH?

Ivan: Yeah, they pay you ETH.
But you can ask them to pay you in different currencies.

## Forensics (1)

So if you want to jump into a blockchain space,
I would say if you are interested of course in the **security**, you can do this.
If you don't want to look into the smart contracts,
you can use **forensics**.
Forensics is basically **finding hacks outside the smart contracts**.
The funny thing is like there was a project with this hack,
which happened five days ago for $26 million.
And this smart contract was deployed I think 2020, or 21, it was 2021.
They never found a bug for five years.
That's why even if you think your smart contract is super secure,
might happen that after five years someone finds a bug.
And that's what happened five days ago, someone found the bug
and drained $26 million from this smart contract.
Funny thing is, I found and I reported him.
But not through the smart contract, we have different... I mean...
Okay, probably here is a good thing to show you something else.

## Aachen blockchain club

There is **Aachen blockchain club**, which is a club here at the university.
It's a small community where we are interested in blockchain stuff
and to play with different stuff.
This is our former president, last semester I was vice president,
so if you are interested in the blockchain stuff,
I would advise you to come here.
We organize different events.
We also do workshops, each... how blockchain works, smart contract, whatever
you could hear in the lecture here.
And we basically worked with different students from the industries.
We participated on hackathons.
We win some hackathons.
Usually those hackathons are really well paid,
so four digits, five digits numbers, easily from one to three days of work.
So if you are interested, again if you are interested,
there is an application until next week, or there are like four or five days left.
I would probably go to have a link,
but you can go on the WhatsApp group.
There is a Whatsapp group.
And here on the link, there should be a Whatsapp link,
and you can join the group and go.
There is a link to the forum where you can participate in the club.
I would highly encourage you to do it because we do lots of **hackathons**.
Hackathons are those competitions on blockchain.
I mean, it is not only blockchain in general, but we travel a lot.
We spoke with cool guys.
The first picture here is Vitalik, the founder of Ethereum.
Here we did some workshops, I don't know what else.
Here we participated, for example, on the global hackathon, we won prices.
And if you are looking for bachelor, master thesis, you can also in the blockchain club,
of course, I can help you with finding a topic.
Because Professor Prince is basically also part of the team and of the group,
so all the blockchain topics are easy to partner ~~with the~~ ~~image~~.
This guy is supervising all the Bachelor theses.
He is now at a startup here in Berlin.
This guy I also supervised finished like few months ago.
He started at 3WC as a blockchain consultant like few months ago.
This guy started a startup, raised lots of money.
This guy won a 6-digit hackathon here from Aachen, which organized.

Yeah, so it is a cool community to be in.
It is nice for you to find talented people who are deep into the blockchain stuff.

## Forensics (2)

So with that, I would say, it is 2:35.
If you don't have any questions, I just write them down...
I can just find where the tickets you can... see the circle...
By the way, do you have any questions?
Yes.

student: How did you find and report the guy that stole the...

Ivan: Here I can tell you.
So here is the thing.
With this guy, with the startup, we created one project where we tried the blockchain transaction history.
Based on this transaction history, we can predict what you are interested in.
Something like cookies and web tool stuff.
You can track your browsing history and everything.
But for web 3, in that case, we tracked your wallet history and we can try to predict what you are going to do.
Are you spending any coins?
Are you doing some hip-hop projects?
You are buying free paper and so forth.
So with that software, we started partnering with the bucket 4.
The bucket 4 is basically where is the water store.
And this is the water store.
Who is it basically?
Who does that?
Website where you can move all the transactions you mentioned.
For example, you have to let the internet into your smart contract.
And then there is a publicly refuted transaction.
After the hack, a few hours later, they published an article where they said this smart contract was hacked.
You can see only where is the transaction.
So we need to do this one.
You know why they don't take the scan.
So they published the transaction hack.
So we kept on two days ago and the guy just trained 26 million from the smart contract.
And the people who found the guy, they didn't find the guy, they just found the transaction hash and they marked it as,
hey guys, this wallet, this is now a hacker's wallet, so please don't interact with it.
Please don't accept any money because we stole the money.
If we have more time, I think it's going to be a problem.
Other smart contracts which are for mixers, which you mix or what it is,
they disappear forever and no one can trace your transactions.
And you can use it in forever.
But the hack happens.
That takes a move.
So first there is the bad thing about this market.
So he stole it but he didn't want to use it.
No, he stole it and he sent it to a different wallet.
So let's see where it is.
So here is the thing he created a smart contract with this transaction.
And once he created a smart contract, he caused the function from the smart contract called attack.
Of course, the smart contract is not audited, so we cannot see the actual code, 25 code.
But he found with this guy from the image, I showed you with this guy, we are now working with both brokers for.
So we found that there was a specific IP related to who looked into the transaction exactly 10 seconds after the hack itself.
So he called this transaction and he immediately, so he immediately goes to the scan and look whether the transaction was successful.
I would just wait one minute because other people just after him, he was the first one.
I mean, he said the first one you cannot assume that he is a hacker, but there is a high probability.
But because he was so stupid, I found out that seven days ago he created another smart contract which is created another smart contract.
Here.
So yeah, 18 days ago he created another smart contract.
Here he created another smart contract which is again identified.
I don't see the code of this smart contract, but he was again ready to open the same transaction hash and he was trying to hack it actually from 18 days ago.
The transaction was reverted probably this smart contract was not working.
They were some problem with the smart contract.
Once he called the run function, he couldn't drain the fluid water.
So he is attacked, didn't succeed.
But again, at the same time, at 10 seconds later, he opened the same transaction hash and this run function from the same IP which is leading to some guy in UK.
But in some time, I mean, once he created the smart contract, during this day, he tried to use different word to create bunch of different smart contracts to fix it.
It was basically a continuous project.
This contract didn't work out.
So he used another word to create different smart contracts and bunch of trials to hack the other word.
From the other word, he was using VPN from Russia, but it was different VPN.
I mean, only one.
So we know that it's a VPN.
We cannot, we are sure that he's not from Russia, but here in this transaction and from the other transaction, he used his private VPN from UK.
I mean, we found his IP and everything.
So we reported it.
There is a website where you can report the hack now and waiting for the teams to confirm it.
We don't get any money because I mean money is already stolen.
It's somehow convincing and finding to convince him to return the money.
Probably the work.
But typically those exploits we don't get any money.
But if you find exploits on the smart contract before the contract, you get what you want.
Do you know what the legal status is?
Like is it legal to do what you do?
No, no, it's a legal.
Of course not.
They reported that.
So basically there is a website.
This is called CO911.
So it's basically security researchers who work together to identify hacks.
And if there is a hack, usually a hack, I would say every month, sometimes every week.
So people are interested in hack smart contracts and still work with them.
If you are really interested in something here, you're still more than a million.
So there is a special data team in the website where in your project owner, you can basically report
and see what you're doing here with RACTAB to trust your partner and report him and report him to the police and that sort.
But this is the problem here.
They don't do it for money because it's more than a minute and a half.
In those websites, we can show you here, like Code for Arena, those are just before the hack.
So before the project needs a hack or report.
And here, these unified projects are white but still not hacked.
So usually those are also hacked but in the future.
And then someone finds the hack.
But if you find it important, you get my thoughts and just do that.
So if these contracts are often like 100K for you to like find a bug,
like how much money are these transactions actually because they need to be way more, right?
Yeah, of course those smart contracts, they usually are like millions, even billion.
They store what they think. I mean, blockchain is...
One of those, if they give you one million, then at least they store more than 100K for sure.
Because they give you around 1% of their creation.
But some of them are hacked from them so probably they store even more.
So if someone does find a hack, the incentive is for them not to steal it because there is this port instead.
Technically, if you are boosting to your smart, you can steal it and no one can find out.
There was a really huge hack, or if you are not real course, probably it's a buy-it hack,
it was hacked from one from February.
It used to 1.5 billion.
Yeah, 1.5 billion.
So this was in February or not.
And they found the guys, but they were not Korean guys, so they cannot win.
The guy was... I mean, I met the guy, the founder of a conference called EATS-Kran.
EATS-C, this is the biggest blockchain conference in Europe,
usually from the blockchain culture.
I actually gave a friend a ticket character, a ticket to the country,
and they called us a ticket character from the conference.
So this is like a good conference, it's close to like 15,000 people or more.
And the guy was there, he hosted a site that only provided for physical and first-rate, right?
Like, if you have taken the store 1.5 billion from the social care,
you survive, you go forward, you are so liquid, you can pay our personal...
It's very nice.
That's all I mean, if you find them, they will pay you easily.
You can ask them anything, they don't care.
Better for them to pay, then they have to.
Hacks are given.
If you just go home to your hacks, they will find the hack, because of everyone.
This is probably the most effective way to do it.
But they didn't find the guys, they didn't find them.
There is a North Korean group, they have no hack.
All their products are related to the same North Korean group, but they...
How can you go to North Korea?
This is a problem.
I mean, they know that it's this group of people who have the authority,
but the area just... you cannot name.
But if you are like this, you pay guys, probably they will find him after a month.
So, that's all I would say.
If you are super smart, you can hide yourself, and no one can raise you,
unless you forget using VPN and other stuff.
You can try to not waste three days, but waste two days, based on your browsing history.
But, of course, you need to be browsing history and transaction history.
If you are super smart, you should know everything, how it's done,
in order to basically don't leave any traces and be in still the money.
There's lots of people who are living there.
But if you come to the studio, I would tell you, don't hack.
Just report a bug, you still have admiration from the whole community.
Everybody will invite you to speak about how you define a hack or what.
This will be... forget what's the matter.
And not on that, not on top of that, once you find a bug,
other people will invite you to look into their smart contract,
and they pay you even if you don't find a bug.
So, basically, they hire you as a freelancer to take over their protocol,
and they pay you visually.
It's 500 dollars an hour, 1000 dollars an hour.
Go on, you're good.
And those people are actually definitely...
and all people who started finding hacks
were cleared on six months of experience with the theory.
In six months, I would say even if they just hashed the blockchain,
they didn't know anything, probably they gave some background knowledge
to the science, I would say it's a bonus, but it's not a mandatory.
And if you look into the... because the hacks are just people who use...
you cannot imagine how stupid have you got this re-enhancing attack
which I showed you in the slide, but skills came out.
If you are stupid, you forget the sequence of the project.
And if you use, of course, this bold profile.
So, just to reiterate, the bytecode of the smart contract
is just possible to...
The bytecode and the smart contract is your favorite thing
that is posted on those focus sources called EtherScan.
For every chain, they give their own version.
This is Ethereum EtherScan, there is a Polygon EtherScan,
or Polygon.
Even on Bitcoin, there was some... what's the...
I meant Google Space.
You can see that transaction, but the difference in Bitcoin
is that you see only transactions here,
because on Bitcoin, you don't see a smart contract.
You can see that they interact with Ethereum,
Ethereum virtual machine, both chains, with your smart contract,
and there is an attack.
And then the owner has to kind of...
put the code, the order of the code, and then everyone can see the code.
Yeah, and then everyone can see it.
I mean, you see all the transactions from this hacker.
Everything he interacted with from the hash, which is this mixing pool.
It's basically a deposit of money.
It's mixed with other people's money,
and then you withdraw the money to a fresh new wallet,
where there is no transaction.
And in that case, no one can trace back that you are the hacker,
because they see your wallet,
and they just see an initial transaction coming from Ethereum
to your fresh new and self-ordered,
and then you just start your normal life.
No one can find out whether you stole the money or not,
because they don't come from the hacked wallet,
and they don't come from the smart contract,
which is...
from the allocation.
I found a link.
I don't know why it is here.
But this is the link to the...
to the...
...faculty encode.
If you are interested, you can share it somewhere,
or you just go to the website,
and then to the order group,
and to find it there.
And yeah, I mean, here you can find a lot of people who...
in forensics, this hack security,
web-to-transactions, and web-to-security.
This is the guy who...
...reported the book.
And he's on the project,
and I think he goes by the fact, or something.
Only from reading the article,
and thinking, oh, yeah, no, a protocol,
which is sort of the same, so I'm from the...
probably they keep the same number of...
So, yeah, you can speak with some cool guys there.
And of course, because everything is paid,
I mean, we got a lot of money from companies like Ethereum,
those trips, which we go to...
...CAM, for example, this one...
...CAM.
ETC, this is for free, for...
Last year, I went to Dubai,
and Abu Dhabi, I was at the Savana conference in CAM.
A year before, we went to Bangkok,
and Taiwan, we went to New York,
and I think in France, we were...
...usually just put it in the ground, and that's it.
And of those companies, we can use the fine-drop,
and it's really important to think with them.
Probably they just made the fine-drop right now,
if you are good, they would just help.
And, yeah, there is another website,
which I can share with you.
This is one of these websites,
it's ur.ceterity.com.
It's a Savana website, it's a competitor to Ethereum,
another blockchain where you can even if you are not a smart content developer,
you can create and tackle designs,
or compete on some content,
and you can earn a good amount of money,
or, yeah, even just 5,600...
...200 or so years ago,
something like that,
but it's not yet.
So, yeah, you can...
which are interested, but you can...
...earn the opportunity to earn,
but it is only related to one example.
Ethereum, but still, I mean, they are nice stuff.
And the biggest hackathon is this gig almost,
which is basically a gig from Ethereum and Role,
and they have hackathons, like, every year,
so you earn 6 times per year,
and basically those hackathons,
they...
they are really good because you...
you find people,
you participate in different ideas,
and you win something,
usually you win over time,
because they give you a wide profit and it has it,
so there is money for everyone,
even if you do something simple,
you find a cool basic video can...
you can also go in front of the people.
But I think the next one is open in January,
this is in US,
but in the coming April,
there will be a nice hackathon,
and you will have a lot of fun.
So, yeah, but this is again Ethereum.
This is some...
...
If you are not a huge viewer,
you are a huge viewer,
but for your experience,
you are almost always huge.
So, yeah, the website, I would suggest,
is global for Ethereum events.
It's one of the super teams for earning some mobile money,
and site credits,
building some mobile change stuff.
Then, if you want to learn programming,
this is smart contract engineering,
you can find the right approach on YouTube,
and, yeah, if you want to hack
or participate in bug bounties,
just go bug bounties on Ethereum,
and you will find this code for an app,
and you will see many times.
And, of course, if you get an impression,
you can actually have a single detail,
the average is a little bit more than that,
and that's important.
But I would also probably suggest
you can just, okay,
questions or...
