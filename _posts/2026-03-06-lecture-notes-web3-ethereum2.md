---
title: "Web3 and Distributed Ledger Technology - Ethereum and Smart Contracts (Part 2)"
read_time: false
excerpt: "For studying \"Web3 and Distributed Ledger Technology\"; content mostly from \"RWTH lecture Web3 and Distributed Ledger Technology\" by Wolfgang Prinz."
toc: true
toc_label: "Contents"
toc_sticky: true
author_profile: false
categories:
  - Notes
tags:
  - web3
  - notes
  - lecture

---

# Ethereum and Smart Contracts (Part 2)

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
I would not say, I think it's based on the...
Every block has a maximum size limit,
so basically the amount of gas you can pay,
so if your transaction will not fit inside the port, it will be eventually reverted.
And if a transaction is reverted, do you get back the gas fee or is it lost?
It's lost.
It's still lost.
Yeah, so basically you lose the money.
Okay.
So, now that you saw what the smart contract is,
like, being that this smart contract is actually compiled to By-Port,
you...
I just generated this image to basically give you a visual explanation
what's a By-Port and what's an Up-Code.
Because on a lower level, I mean, when you write this smart contract in the street,
you write it by functions and payment everything,
but on a compiler level, we execute this By-Port.
And these By-Ports are basically a version of the Up-Codes,
and the Up-Codes are basically those operations where we need to pay gas fees.
So, it might be a little bit confusing,
but we pay...
Our payment for executing or for deploying a smart contract is based on how many Up-Codes we use,
and those Up-Codes are later on converted to the By-Port itself.
So, here this is some Up-Codes, like push.
Push is basically pushing to the stack,
and yet, for example, adding one number to another number,
I think on the next slide I'll give you an example.
But you should remember here that the Up-Codes are basically the codes which are paid,
and based on the Up-Codes, you can calculate how much gas fees you need to pay
for deploying a smart contract for calling a certain function on that smart contract.
And, yeah, I found out that there are more than 140 Up-Codes,
so based on what kind of operation you do on Chink,
depending on how much money you need to pay again.
We think about this example you showed in the previous slide.
If we store a value, like, imagine it as basically like an API that I can call of a server,
like, it's in a model that makes sense.
Like, somebody calls it and says, okay, I want to store a number 42,
and then somebody else calls the gather function and says, okay, I now want to receive the number that's stored,
and you get 42, and if so, where is this number actually stored?
It's a blockchain-based server, not on the machine-based server, but actually stored.
Yeah, so basically you execute it on the node,
and you see the virtual machine node, which is running the virtual machine software,
or operating system in our case.
But every time when you call, for example, store number 5, I think,
so then it's like, you can just add here, here is a good explanation.
There is a function called add, so basically here in that case, I'm summing a group with, right?
And this is the needy, this is now the opcodes.
Opcodes is push, add, and store, and present.
And when you're on a computer, you get like, this is the card drive,
you can also add RAM, here you get also a memory storage,
and the stack is basically the lowest...
Okay, let's start first with the storage.
So whatever you store on the storage, you store it on the blockchain,
so basically you get access data on the blockchain,
I can retrieve it from the blockchain source.
The memory is basically only storing information inside the function,
so when this function is completed, then memory is deleted,
and stack is basically a small operation, when I just store...
If my function is A to B, and then I say, the result of A to B should be summed with C,
A to B, the result of A to B is stored on the stack.
So this is a spacing where you store information, but for a really short amount of time,
within the function in the memory.
This is memory of the function.
So if we have an example of the number storage, and people like...
If I call it, I say it's a 42, so 0 to the 42, like all the time,
then all these different states will be stored on blockchain forever.
Yeah, so basically what we'll do, we'll hold this function just normally,
this function will be executing this op-codes,
so basically take the first place, A, and the second place, B, then add them together,
then store this information.
This is just storing it for the function, for the lifetime of the function,
and then return, but if we want to store it for the lifetime of the smart contract,
we just need to add a store, which stands for storage,
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
but the thing is that you hear this, if you go to this Merkle tree for a while,
and these other transactions are stacked in the tree structure or something like that,
all those transactions are part of this tree,
and all those transactions are basically changing the state of the tree
and changing the state of the blockchain,
and whenever I need to retrieve information from the blockchain,
I just go and ask, okay, this smart contract, it gets a storage,
I told you that there is a storage, just tell me what's the number stored on the storage source,
and it's a one or two or whatever storage storage is,
and then you can retrieve the number which I stored on that tree.
But this number is also correlated to the state of the Merkle tree chain early.
The only thing that's confusing me is that if we have an actual application running there,
where every stage is basically stored in the blockchain,
doesn't that generate an incredible amount of data?
And that's why it's constantly increasing the size of the blockchain.
Every time when you save some information, you basically save some more information on the blockchain,
and you make it bigger.
That's why last time when Professor Prince was talking about retrieving code on the box,
he said that the later you go with the box, the slower it becomes to retrieve information,
because there were more transactions, more interactions, so everything becomes slower and slower.
But think about it just as an SSD card which is shared among all of us,
and whenever we send information to this hard drive,
basically everyone has access to this hard drive, everyone can save information there,
and everyone can retrieve information.
And this hard drive is not one hard drive,
it's just a copy of this hard drive that's saved from all nodes running from the blockchain.
Whenever I change something from my node, from my hard drive,
I basically say, okay, I changed the state of the blockchain,
please synchronize all your other nodes around the world,
that we get the same state, the same information which I saved from this platform.
What do you want to answer?
Thank you.
It's confusing, I have an idea, probably I also didn't get it from first five, ten times,
but the longer you use it, the easier it will become.
That's why I would say this is not something you should understand from the beginning,
I don't think even the traditional testing application is something personal,
but for example, don't take this for real,
I just wanted to explain how it works,
so that you can talk more knowledge about how it works.
And yeah, as I explained on the previous slide,
those opcodes, they are different types of opcodes, more than 140,
and those opcodes, they have task fees,
so basically depending on what we do on the blockchain,
we call different opcodes,
and these opcodes we need to pay for example, for push,
and they have much trigger, three units of that,
and for pop, for adding, quantifying, saving for memory,
saving for storage, jumping, holding data like we have,
different pricing, tax, how much we need to pay for the opcodes.
And yeah, pretty much that's the, on a war level, on a combined level.
So now going again to the smart contracts,
as I said, it's written in utility,
it can be also the second most famous language is Bifar,
both of them are just object-oriented languages,
like Kawa and Kawa script, I would say,
the utility compiler is called S-O-C,
so it comes with a compiler,
this is the compiler which is combined with a bytecode,
and that's combining the utility and bytecode,
and then we have, what do we get?
Yes, we can see code smart contracts.
I'm not trying to get you to know,
every time when I call it a transaction,
this compiler is basically running from the virtual machine,
which is the OS of the node.
Yeah, do these prices, these gas prices change?
No, they just fix, but the guy's from Ethereum,
think about it, can they change it?
Yeah, they change the earth,
of course, updates of the EDM and updates of the sweet opus,
so eventually at some time, probably they can change it.
I actually never check whether they can change
in the future, but eventually they can reach the content
where they make, for example, storing more expensive things.
But the gas fee rate does change all the time?
Gas fee rate, yeah, changes in a minute there,
depending on the usage of Ethereum,
so if there are lots of people right now using it,
there will be lots of calls into the work space,
but this is probably, if it's changed,
it's probably changed on a few years' time.
Are these gas prices for deploying the contract?
For deploying, basically for interacting with the smart contract,
if you look at this, this is a function,
this function is basically calling those opcodes,
so basically it's calling push, it's calling hit,
and it's calling can store.
And those opcodes, they have gas fees,
so whenever I call this function, I know how much gas fees there are.
When I deploy it,
deploy it, you deploy it, you change the user.
Another one, right?
Yeah, deploying the smart contract,
you pay only for the opcodes used for the deployment.
But if you use the, after you deploy the smart contract,
it's already existing, so you get now those functions.
And then, depending on the functions,
you can go to the bytecode and calculate it.
I mean, the wallet itself is calculated automatically,
but if you want to go one safe deeper,
you can also just look into the opcode
and say, okay, the opcode costs three units of gas,
do I pay three or do I pay more?
So the example with the edge is doing the five opcodes,
that's for executing it,
and if I want one to execute the smart contract,
I would pay the gas fee for those five opcodes.
And for deploying it,
for deploying it, for probably the deployment,
or not another set of opcodes.
The point is basically when you create a smart contract,
you generate your compiler,
you generate the opcodes for deployment of the smart contract,
and they will be probably some other.
This is only for the execution, this is only when I call the function.
But for the deployment to be something,
you don't pay this for the term.
So the gas fees work like, for example,
the cost of the story costs like $20,000 I think,
so it's $20,000 times C,
and the C is that when the gas price changes,
or like, what is C?
Like any constant?
Ah, the constant, yeah, exactly.
So the cost of deploying such a smart contract
depends on the constant of the smart contract.
Exactly, so the longer the bigger the smart contract is,
the more operations I get,
and that means the working group is more complicated,
I need to do more.
Does it just depend on the size of the smart contract,
or also on what it contains?
When you deploy it?
In the case of the onus, I never moved that deep.
I would assume both, but I'm not sure.
I'm not sure.
I need to deploy it.
Definitely the size, because the bigger the smart contract is,
the bigger the exchange, and the priority, everything.
But this is for sure,
but what else?
And what's the point of the same smart contract?
Yeah, you can deploy it, but it's not in the same work,
it will be in different work.
Or, yeah, in the same work, work with different transactions.
So who cares if the gas is in the work,
if people deploy it?
If you deploy your own version, it will be the same version,
but you pay from your order.
I can copy the same smart contract,
which you deploy, and I can deploy from my order.
So eventually, you will get the same logic smart contract
with different addresses,
but I will pay for my own, and you pay for your own version.
So I just upload in an image, for example.
You upload the image from your computer,
I have to upload it from my computer.
There are two instances of this image,
leaving from internet, but one is uploaded from you,
one is uploaded from me, so the one I uploaded,
I pay for it, and the one you uploaded, you pay for it.
The same constant code with two different contracts.
Yeah, but they get two different addresses.
So if I deploy the same important,
if you deploy, we will eventually,
you know, get two different smart contract addresses.
So if the smart contract involves
me telling one place, someone else.
So the personal place, the gas fee,
is the person who deployed the contract.
Yeah, but if you remember again,
you cannot pay the gas fees automatically.
You need to, every time when you pay your monthly fees,
you need to call the smart contract,
hey, please pay to my landlord,
my apartment fee, yeah, my rent.
In that case, your wallet is calling the smart contract,
which is having a function, pay, render,
pay your rent worth or whatever,
and then you pay for the execution of these transactions
from your wallet, and if the smart contract has balance,
probably it will send the money to the rent point.
But in that case, I don't,
if I call your smart contract,
I would pay your fees, I would pay your rent
using my wallet fees.
So think about, we have the same version of the smart contract,
every month you call a function of that smart contract
to pay your landlord, and every time you call this function,
you pay a small transaction.
And if I call this function on your smart contract,
I will pay the transaction,
but I will also pay your rent from the smart contract.
If the smart contract has a balance,
you're not to pay for the rent,
because if the balance is zero,
then the execution will revert,
I thought it was atomic,
if there is not enough money in the smart contract,
it will just revert,
I would eventually pay the fees,
but the rent won't be paid,
or there is no money in it.
So you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
you're not to pay for the rent,
so again if we go back to the sample,
we have the Progmo summertime,
which I sent this is a bespot version.
And we have the name of the smart contract,
and then we can give different types,
like, as I said,
we can give unsigned into JET stroke,
signed into JET stroke,
So I said everyone on blockchain has an address,
so we need a type of address to determine
where the transactions are coming from.
Those are KF bytes, and they are different types.
I would say this is just informative,
so you know what to expect,
but depending on what smartphone that you want to create,
you can look into different types.
I am very important, there is no thanks
on all the smartphones.
We use this, I think it's called
an epoch, you know, something like this,
which is picked number from 1917s,
which is representing our current time date
and seconds.
What else do we have?
So it's very important that we get different variables,
so we get contract variables, functional local variables,
and blockchain global variables.
And most interesting are the blockchain global variables,
which means that those variables are publicly accessible.
So usually the person who send the transaction
to the smart contract is called message-rock sender.
Everyone has access to this.
We can see the value, if we deposit, for example,
the example from before with the rent.
If we send money to the smart contract,
we, there is a message-rock value,
basically writing the value of the money
which we send to the smart contract,
and it's stored there.
And we also get walk number,
because every transaction eventually will be saved
from some walk, so we can use the walk number
to reference where the transaction was sent.
And there is also walk time sender,
so basically, again, in this unique state of time,
we can determine what was the time and date
of calling the smart contract.
We also get just simple operation like arithmetic,
with minus, you know,
Teo, and comparison, logical and forward-in-patient assignments,
just like normal, normal, okay?
How is it made sure that the blockchain global variables
are in the main space?
Is there a transaction ID in front of the message-rock sender?
So message-rock value can be same.
If I send you one, you can do all the same value.
Walk number is always unique,
because walk number basically is
the end of the transaction, as you always prefer.
But if I want access to walk number,
from the transaction, you can see
the areas on the walk explorer,
you can see the open, they are available.
Those walk explorer is the website
where everything is visually accessible.
I mean, that's probably the main point, I get it,
but if I want to, for example,
access them from some...
Yeah, it's the best...
...the next thing I have a unique name, no?
Yeah, it's walk-off number.
So if you write a smart contract and you say,
I want the walk-off number to be bigger than that,
and that means this function can be called only
from walk number, which is in the future,
and in the future.
This is the example where you say,
can we limit it, for example,
to not do the smart contract after this session?
We can say whoever sends a transaction
to our smart contract,
and this transaction ends in a walk number
that is later done in the specified way,
when it's the deadline of this lecture,
then it will be rewards every time someone tries
to send the transaction to the smart contract
after the lecture, it will be rewarded.
And I can, from the site, the smart contract?
Yeah, I can access message.sender,
and you always will need this message.sender
from the guidance column.
Yes.
Okay, because there are probably thousands of messages
to send that are removed, right?
Yeah, okay.
That's why it's called a warning.
The smart contract just compiles it without any errors,
and then sometimes someone is going to just take it.
So sorry, those values are then in the global namespace,
and they are performable?
Yeah, yeah, performable.
What else do we get?
We get loops, but they are not really a good option
to be used because the longer the loop is,
the more gas fees you need to pay.
So eventually, if you end up creating a smart contract
with a big list where you need to loop
to the items and the list,
you are basically saying,
hey guys, whoever calls my contract
needs to pay a bunch of gas fees.
Some people won't be relegated before the smart contract.
That's why we use mapping,
on the next slide.
We use arrays, I mean here is the creation of arrays,
how you can initialize it,
then creating element taxes and elements,
it's just three-day old stuff, yeah.
What's the difference between the two initializations?
Not just different type of initialization,
I mean both of them do the same thing,
just different syntax.
You can choose.
I usually use the first one, but you can use the second one.
So then it doesn't change where it's stored.
Yeah, just different.
What else we get?
We get the emulation, yeah, I'm not sure.
In all of them, we have structures,
so basically like a struct,
and each struct can also be initialized.
In our case, we create a struct called student,
student struct, there's a name, semester, and grade,
so we can get those objects inside those elements
inside the struct.
And then we get this mapping,
which I told you it's very popular in the smart contract
because it costs lots of money,
so usually we get a mapping.
The mapping is between address,
usually between address and something else.
For example, if I'm the owner of a smart contract,
my address will read to, for example,
the owner balance will be 10, so it will read to 10.
If you are just user, maybe you put your address
and you get five things.
So this is how you map your address
to some integer or string or Julian or whatever.
You can get a nested mapping, which is a mapping,
inside a mapping, but it's a little bit more complicated.
But this is usually when you create a list of elements
and to say I want this student to get five zero,
you don't go to the loop of this list,
you create a mapping so that you can discuss
the repeat elements on the smart contract.
Okay, so what's that?
Yeah, so now we go to the functions.
The functions, they get different properties.
Some of the functions are called view,
which means that if you call this function,
you retrieve, you read the load from the blockchain.
Some of the functions are called pure,
means that they are not even reading from the blockchain,
they are very rarely viewed, I would say.
And if this function doesn't have a property
where you are pure, it means that usually this function
can save information, so write and treat from the blockchain.
Usually the syntax of the function looks like this.
So we have the function, then name it,
then we have a parameter or a simple parameter,
then we have this property, whether it's view or string,
and that's the function that we have here.
So the prefer string, maybe it doesn't prefer string.
And there is this public feature coming from the next slide.
It is basically saying who can call that function,
because not everyone, not all users
can call that function function.
This is actually, I would say,
in some reddit, because it's my expression,
to say what's the different between public and external,
what's the different between external and private.
It's pretty steep, straightforward,
and you don't, from the name of the function,
you can derive it.
Public means that it's open for everyone,
can call this function, external,
and then you can call this one,
smart contract can call this function.
It's publicly available and open for everyone.
Then we use external functions,
which is somehow like public,
except it cannot be called within the contract.
So the contract is sealed,
maybe there is a function called add,
and there is another function called subtract.
Those two functions cannot call each other
within the smart contract.
Then we get another function, which is called internal.
In that case, you restrict, right?
No one can call, no one from outside can call your function.
Only your inherited smart contract can call them,
or the smart contract itself.
Then you get a private, which is the most restrictive function,
and that is this function can only be called
from the smart contract itself,
even the derived smart contract.
Because here we get an example,
like there is an inheritance,
like an NTF, or another smart contract,
and choose a smart contract,
which is inheriting all the properties
from the another smart contract.
In that case, internal means that the children
can call the function from the mother,
and in private, it means that the children
can call only functions inside the children.
For the external, with the explicitly prefix
of this keyword, is this,
this is like in Java, for example,
where you refer them to your own method?
Yes, yes, exactly.
The actual is very similar to Java.
But why does the external then exist,
if it can be called from the contract,
when you really do what this,
I mean, this is again specific situation,
I would say just before it,
just if you get this question,
say external smart contract cannot be called from outside,
can be called only from outside,
not from the contract itself,
that would be sufficient.
So basically, it means everyone can call it,
exists as smart contracts,
the smart contract itself or other smart contracts.
And so this means that with external,
it can only be called by its own contract
and externally all the counts,
but not following smart contracts.
Not from own, from only from external,
from what it's from other smart contracts.
But not from, because there is a function, for example,
do you have a smart contract with these functions,
one, two, three, four,
probably this function needs to call this one,
and this needs to call that one.
On that case, you have function one,
is calling function two,
in that case, you cannot.
Because external means that function two
can be called only from outside.
But if I, you cannot call it from here.
Three things to call with this?
Don't care about this, it will become too confusing,
just think if you can be called only from outside.
It's not going to, don't worry.
It will be just more confusing.
Just think that public, public means everyone,
if everyone can do everything,
private means no one can do
only the smart contract they want to.
And then think external, which is external,
only from outside the smart contract.
Internal means probably it's internal,
but internal in a way that the smart contract
derived out of this parent, mother example.
Okay, so what else do we get here?
We get events, which is something like a notification.
We can create an event,
and every time an event is triggered,
we get notifications,
you are like mobile phone and apps.
So every time we, someone creates a transaction
or everything, we can subscribe to an event
that is important for someone just paid
to the smart contract,
or someone just called function A from that smart contract.
So think about event is just a notification.
This is, I think this is important.
Then we have here different types of structures
we can create in the smart contract.
First we have a constructor,
which is initializing function like C for example.
This is a function which is called only once
when the smart contract is deployed.
And usually this mark, this function
is determining the owner of the smart contract.
So if I say in the constructor owner
is equal message.sinder, which is the global variable,
means that whenever I create the smart contract
and deploy it, I automatically assign myself
message.sinder in the owner of the smart contract.
And then I can give different functions
where I can create a modifier,
meaning that only the owner of the smart contract
can create, can call this function.
For example, this one.
I can have a special function called destroy
and then want to destroy my smart contract.
I don't know other people,
even though it's public and everyone can call it.
I just want to restrict that access
that only me plus an owner,
which is derived from only owner modifier,
can call this function and destroy the smart contract.
Otherwise everyone will be able to call it
and just destroy the smart contract.
But again, still destructive.
I think it's not very many more since last year.
So those are just examples how you can use only owner
but also the service contract.
Yeah.
What are the function names
like these functions are actually predefined, right?
Functions you show destroy, this is very fine.
You can call it destroy, right?
For example, you can change it.
You use the term only.
Ah, this is predefined.
The modifier, the red thing is predefined.
This is predefined and this is predefined.
This is like initializing function.
And the red then it's part of the sweet.
If it's good, then okay,
in that case, it's confusing,
but yeah, you can think about here,
the name of the function.
This is try functions asymmetrically the same, right?
Yeah, this here, we use this requirement.
We have basically required the matrix of center
in the overall variable.
It's equal to the owner.
The case with owner is this owner
which we defined in the beginning
with the requirement of the variable.
But they will the same.
Just here you get the requirement here.
And here you have the requirement inside the modifier.
Usually this is for saving for spacing
to create a modifier.
And instead of just writing here the function required,
you just put the modifier name
and you know that this is already defined.
This is the, I don't know,
some point of length,
underscore.
Yeah.
You need it because this is the case
for everything.
I will work.
I will work.
I always need this.
Okay, by the way, do we have to do better?
Yeah, of course.
Because I'm not sure.
I don't know how long it is.
The room is booked to three,
but the usual lecture time is later.
Okay, I'll try to go past those.
Yes, lots of stuff here, but I can do this.
This was just an example of smart contracts.
So going again backwards.
Smart contracts are probably boring.
And then you'll find a useful situation
where you create a smart contract.
Usually most of the smart contracts
are deployed for creating some kind of projects.
On-bop chain, 90% of the projects are related
to an initial stuff,
for example, tending money, investing money.
They were related to money.
But you can also create smart contracts,
which are just, I created for example,
a smart contract which is saving
between the different smart contracts,
but it was saving the,
you know, if you buy for example,
some expensive goods,
they get this unique identifier number.
And usually, some people can take this number
and produce fake, for example,
watch your check for dealer,
with this same identifier.
That case, you can avoid this problem
because you can create a smart contract,
which every time there is a new fake produced
or new watch, this watch has a unique identifier
and this identifier can be stored,
for example, inside a smart contract.
So whenever someone buys this smart product,
this watch or a fake or this watch,
they assign this identifier to this wallet.
So if a second big is produced
with the same word that identifier,
rather with the same product identifier,
in that case, the user who is buying the big
can go on-chain because again,
everything is totally clear,
they can go to the retail or official website
and compare, okay, is there another item
with this identifier already sold?
If there is, in that case,
means a person who is selling this big or this watch
is basically malicious, he's fake,
because he's selling another big with the same identifier.
Eventually, you can prevent fraud works.
Yeah, fraud works.
I mean, this is if you think about some off-chain.
We also created a version of a smart contract
where every time you open a medicine,
you give this paper with the instruction
how we take the medicine.
And it's not a work, I mean, just paper, you need to read it.
We created a version where you can scan up your code
and you basically take the whole instruction
of this medicine online and you can read it, for example,
from your phone or from your computer.
And now here comes the thing.
I can just store it on Google Drive,
I can store it on my server wherever I need.
But what happens if at some point there is a hack
and someone changed, for example, the instructions
to instead of take one pill thing, take five pills,
you're out, maybe dangerous for the person.
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
then you should ask the patient,
should they really trust these instructions
from the medicine or not?
This is just like examples of how
they create smart contracts
which are not related to financial.
One last example I give you
and then we go to the financial progression.
There was a big project where people in Denmark
were producing those wind turbines
for creating electricity.
In those turbines, they kept on the break very often,
especially when there is strong wind or some,
yeah, bad weather conditions.
So the insurance needs to pay for the repayment
of the broken thing.
But the insurance pays only if the guys who inspected
the turbine did their job correct.
And how did they do it?
They kept a big, how do you call them?
Like this gadget, it's in German, it's real one,
it's like torque, I can pick device which is just,
yeah, torque, I can't put that.
So they need to basically make an inspection
with a certain power that every bolt
and every basically is done correctly,
which is every bolt is put correctly
and with certain amount of power.
So that in case there is a huge wind and everything,
if it breaks, you can always say,
okay, but this guy has to be able to do the proper inspection.
So this device is a booted device
which I connected with the blockchain
and it sends the data to the blockchain.
So if in the future this wind turbine breaks down,
we have the data from the inspector,
save on chain and the insurance company
can go to the smart contract and state all the data
which were stored on chain and if everything was great,
they would just be for the repairman
because they're sure that the repairman was in the right place.
This is example of creating smart contract
outside the financial.
But usually also, as I said,
most of the smart contracts are inside the financial
in-sync division, like I want to make money
and work with the sum.
Yeah.
What does message.sendle.call.value do?
Which one?
This is the sum.
The left near the bottom message.sender.call.value.
Just a last one.
Yeah.
This one.
Yeah, this is basically, yeah, I will explain how it works.
So this was one of the first hacks
kept on Ethereum in the smart contract
where people deployed a bunch of Ethereum
inside the smart contract and they were early in the morning.
And at some point they were a bunch of Ethereum,
so I think even in percent of all Ethereum
in-sync division where that smart contract
captured the guys from the other team
who created the smart contract, it's really a nice project.
But there was a small bug,
and the bug is actually here.
Here's a problem where think about you have a smart contract
which is storing some Ethereum inside the balance
and then you have this function called withdrawal funds.
And withdrawal funds is basically asking,
okay, who is the sender or who is calling this function?
Message.sender.
And this is balance which is a mapping.
Yeah, this is a mapping.
Thus he has more,
because everyone who posts this money has a balance.
So if I post one Ethereum, my balance will be one Ethereum.
And think about this is in wave.
It basically has more units of Ethereum,
but think about this is one Ethereum just before.
So if I call this function and I hit previously
posted one Ethereum to the smart contract,
and if I call withdrawal funds,
usually what should happen?
The smart contracts should check my balance.
So in this guy has deployed one Ethereum.
If he deployed it,
if he wants to, does he want to withdraw more money
than I'm allowed to withdraw?
For example, there is a hard cap per transaction.
I cannot withdraw more than one Ethereum,
but I want one.
So in that case I satisfy this requirement.
Then there is another requirement.
Did I want to withdraw less than one,
we for more than one, we can also ignore that.
And if I meet all those three requirements,
then the smart contracts okay, everything is fine.
Now just take the sender and send him the money
he wants to withdraw basically the one Ethereum
which I look here.
And when I receive my money,
then my balance needs to be complete.
So basically now I don't get one Ethereum anymore,
I get zero.
Of course my balance is one minus one, which is zero.
And then they store my last transaction.
So everything looks fine, right?
But there is a small problem because the transactions
Ethereum they literally execute in the way
they are written here.
So there was one guy who said okay,
there is a problem, he identified this book
couple months later and he created this attack
smart contract which is really simple.
I mean just one contract for the attack.
There is a basically a structure that you don't need
to know this, but what is the attack actually doing?
It says okay, I want to withdraw my deposit.
Basically first he creates a deposit
and then he wants to withdraw the deposit.
And then what is he doing?
He basically has again here to deposit one Ethereum.
And then he says now when I deposit my Ethereum,
I want to withdraw it.
Think about it, there is no withdrawal time,
everything is satisfied, so he immediately deposit
and after the deposit he wants to withdraw the same deal.
And usually I know too the smart contract
can receive some money.
And in order to receive some money,
you need to have a function called payable
meaning that I can send money to the smart contract.
Otherwise I'm not able to send money to the smart contract
and transaction will be over.
So this guy just said okay, I will deposit one Ethereum
and then I will withdraw this deal.
But whenever I try to withdraw this Ethereum
and I reach to this point here,
and try to get my money.
The money will be sent, so think about one Ethereum
pure point here, the payable function.
Then the balance of this smart contract becomes plus one.
But this function has something else.
Like whenever it receives the money, it doesn't stop.
It just calls the same withdrawal function one more time.
So in that case we send the money, we go here,
we send the money, but we don't reach to the update of the balance.
So essentially what is this attack link?
It's just calling again the withdrawal function.
It checks again the requirement, it satisfies the requirement,
then send the holiday.
Then does this like in the loop,
probably there are hundreds of Ethereum here,
then withdraw all the money,
and when the balance is basically,
I mean here doesn't have more Ethereum to withdraw,
it just updates the final status
and this guy ends up stealing 100 Ethereum
from the smart contract despite the initial amount
before it withdraws.
And how does it fix?
The fix was really simple,
you just need to swap the ones.
So basically you first need to update the balance
and then you need to send it.
I mean at first it doesn't look really problematic,
but yeah, it can be one of the first and largest assets.
And everything is fine as I said,
you just need to swap the basis of the balance.
How much did the interest open?
I will.
Just Google DAO half 2017 or something like this.
Like it was our turn.
Is this some kind of concurrency?
Yeah.
Because I feel like if you send,
if you record this with draw hands function,
100 times in the same second or in the same moment,
then also you would have the same problem, right?
Because you check it once in the beginning
and then in the end you end up...
Yeah, this is actually the problem,
I would say more or less with Ethereum,
because in that case this compiler version,
I think right now the different compiler
is checking everything, not sequences,
like one by one, but at that point
this compiler also doing the...
I think there is now by the code of convention.
This is called re-enterncetaph.
Google re-enterncetaph.
And there are other attacks which I would say
we will skip them because we don't have much time.
I will just briefly give you explanation on the left side.
You see over 4 attack,
which means that we have a UIN, for example,
which is storing numbers between 1 to 56.
You can over throw, if it surpasses to 56
it will go again from 0.
So we can have this attack.
We can also have another attack where...
Here in that case,
we are looking for some word.
In this word, it basically is hash
which equals to the hash version of this word.
And if someone finds a word that satisfies this hash,
he publishes this word here,
he should get 100.
But in real world,
there are people who are basically observing an input.
And if someone finds that you found already a solution,
you will just create the same transaction,
there will be a little bit more gas fees
and QO from running.
So basically without any effort,
he will use your solution to get the money.
This is called front running attack.
You won't avoid this.
But I would not explain you how to avoid this
because it's really complicated,
but there are other tricks out there.
So I would literally skip those stuff
because it's not really cool,
but I will show you like a brief example
of how you deploy a smart contract.
We're going to do it before the end.
I want to show you something else which is cool.
So if you want to learn smart contract programming,
I personally did it from this website
called Smart Contract Engineer.
It's a nice website where they have different challenges.
This guy also has a bunch of YouTube videos
so you can watch in cases in everything.
Right now, the contract is back with all the teaching class
because it's for Ethereum,
but if you want to deploy smart contracts,
for example, other teams,
but one usually uses different languages.
Some of it is Italian or Russian.
Yeah, you can use that as well.
Now that you know this website,
I just took an example,
and the example is this one here.
So you can deploy the smart contract using YouTube.
There is a really cool online compiler
which is called remix.
Remix.etherion.org.
I would say just write it down
because professor asked you to use Amazon
to extract easier free to do.
And here on the left side,
we have a compiler and we have a learning device.
So this you need to balance.
Once you have a smart contract,
for example, I have a smart contract here
which is you can check what this smart contract is.
Yeah, so we have an owner.
Maybe I need to put it somewhere.
So we have a smart contract where
I just say when I deploy my smart contract,
I want to be the owner.
So the guy who is deploying the smart contract
will be automatically sent to the owner.
And then I want to have a function
where probably in the future I want to change the owner
of this smart contract.
But I want to be sure that the owner is not address 0.
address 0 means that if you assign the owner to address 0,
no one has access because no one has access
to a word called 0.
There is no private fee assigned to this.
And yeah, if I want to deploy that,
basically I have an option here on the left side
which I use, I can see different compiler version.
Usually, as I said, if you use compiler,
which is about 8, it's 3dc.
I compile it.
And if there are no errors, here you see the green tick.
That means it's ready to be deployed.
Then I click here on the button below
which is deploy and turn the transaction.
I deploy the function as a result.
I can choose the name of the smart contract
because think about here I can get
which post-mart contract is on the file.
I choose to deploy the smart contract called allnobov.
And here on the environment, I can choose
whether I want to deploy it locally.
It means using remix.
I don't need any money.
Or do I want to deploy it using my metamask wallet?
So here on the browser station, I give two wallets.
One is Panto, one is Vana.
I think they also support Ethereum and metamask.
So I said, okay, I want to deploy this with my metamask.
So I will pay some fees,
but I want to see my smart contract on-chain.
If I don't, if I don't get money,
I'll just use the remix virtual machine.
So it means that it will simulate a compilation
and deploy my protocol on the mix,
but you don't see it on-chain.
Let's say I deploy it now.
I'll deploy it.
I create a new hash here.
Metamask just pops up at the end.
You hear this smart contract.
You need to pay network fee,
then the maximum gas fees.
Okay, well, I agree.
And I pay whatever metamask is calculating
for my smart contract.
And here now the smart contract is pending.
I can do it on Peter's app.
It will take some seconds.
So you know, pending, over here, in the middle.
So once here, it stops seeing pending,
I will eventually see this is my wallet
and I will see the smart contract address,
both of my contract will eventually deploy this.
Okay, official website.
Okay, it was deployed.
Now I give the smart contract.
It is blank.
You don't see anything.
But if I click here on the contract,
you'll see the buy code.
And because you don't want to see the buy code,
because you don't understand anything,
we just want to verify.
But the thing is only the guy who hears the smart contract
can verify it because I know which compiler did I use.
In that case, I used 3D compiler.
Then I need to check which version did I use for the compiler,
which is here, 3.31.
So I go to this page, 3.31.
I hope it's this one or the other one.
Then I click continue, 3.
And I should...
Yeah, I should put the smart contract in the code.
Here, and if it's verified,
it should be verified what is working on.
What is working on.
What is working.
Think about it working,
then you should see basically the same code here.
But you should see on the address of the smart contract.
On the text, which is here.
So here you'll see the code.
This is like this.
So think about this is, again, this is the website
where you can go and find smart contracts
who can enter their code
and interact with the smart contract directly from the wallet.
I mean, in order to interact, you need to connect the wallet
because again, for every transaction you need to take piece.
But eventually, what you should remember is
that if you want to deploy a smart contract for the first time,
I would suggest using this one, a remix.
You just copy a smart contract,
you decide what you really want to do,
you compile it and then import.
I mean, it's literally one to two minutes work.
Pretty sure you could be easy to do.
Any questions on that part?
Because I know you were expecting something fancy.
How can I create a smart contract?
I mean, nowadays,
strategy became creating on the darkmail contract with me.
I would say I spent three years on creating smart contracts
and now it's the level that strategy is beating me sometimes
if you want to compete.
So I would say it's good to understand how it really works.
I would say that it's always nice to understand
what your smart contract is doing
because whenever you deploy something
or you use it in production, especially with money,
you need to be sure that your smart contract is not going to be hacked.
And coming to the hacks,
I would say that you lose a lot of business
for me showing you that
if you're interested in the blockchain stuff,
there are pretty cool places and websites you can explore.
One of them is Code for Arena,
which is I started this journey like probably two to three years ago.
There are a bunch of protocols or projects for companies
which are deploying their smart contracts
and those smart contracts, they need to be audited.
And those protocols or those companies,
they spend a lot of money for using the smart contract.
And I would say if you go to this Code for Arena website,
there's a contest, you can audit smart contracts
and you earn really good amount of money.
And I would say six billion per year is kind of to be able to do this.
There is also a huge opportunity for...
How much money is this?
Six, seven billion.
I know I'm going to get six million in one year.
Okay, from this one and...
And you earn this five.
So here you find protocols which are already void,
but you can still look into the smart contracts
and you see the big marks on this.
This one is paying one million for some clients.
Usually the bugs are critical, medium and low probability.
So you can find critical bugs,
you pay up to one million into high, medium,
but they only pay like two hundred thousand and more for this.
But from this website are four project that are wide,
there are thousands of projects.
Some people when they do, they just look into hacks
that happened in the past month.
And because most of the hacks, they can happen in multiple protocols.
So if they identify that the hack happened on some protocol from here,
they just look for similar smart contracts
which are using similar functions for similar idea
and they just report the problem if it exists.
In 1991, students, I supervised with virtual art teachers,
he found a bug from here.
I think it was awarded, I think, five thousand or something.
And he didn't really find the bug.
If you get an article about this bug, he said,
no, but I know another project which is doing exactly the same thing.
Let me see if they are called basically the same.
And it came up to be asked to be a smart contract with some serious smart reports.
But he reported the bug and then I came to this user and it was awarded.
So if you are putting solutions, I would say you can look into this unified.
If you want to look into a smart contract which has not yet been production,
it's this code for arena which is basically projects
for deploying the smart contracts to report production.
And here in that case you are competing with other people.
For one contest, for example here on active projects,
there is currently this code, here but it's rest, it's not ready,
you see the current address, but the past,
until then under the agreement,
so there are probably like a thousand people here competing
and eventually they will change $56,000 to all the people who report the bugs.
But here you are sure that there are bugs.
Those people are always spent.
It's the matter of how much share from this pool you get to see.
The more bugs you find, the more money you get to work.
Next, you just pay the material.
Yeah, they pay you to see the material.
But you can ask them to pay you to see.
So if you want to jump into a blockchain space,
I would say if you are interested of course in the security, you can do this.
If you don't want to look into the smart contracts,
you can use forensics.
Forensics is basically finding hats outside the smart contracts.
The funny thing is like there was a project with it,
which happened five days ago for $26 million.
The smart contract was before I think 2020, so it was 21, it was 8 of 1.
They never found a bug for five years.
That's why even if you think your smart contract is super secure,
my team came under the after five years someone found the bug.
And that's what came up five days ago, someone found the bug
and drained $26 million from the smart contract.
When it came, I found that I reported him.
So through the smart contract, we came different.
Okay, probably here is a good thing to show you something else.
There is a second blockchain book, which is a book here at the university.
It's a small community where we are interested in blockchain stuff
and to lay the trust of.
This is our former president, last semester I was five years old,
and I was interested in the blockchain stuff.
I was invited to jump here.
We organized different events.
We thought we would workshop each blockchain world smart contract
with the smart contract here, we are in the lecture also here.
And we basically worked with different students from the industries.
We participated on hackathons.
We win some hackathons.
Usually those hackathons are really well paid,
so four digits, five digits, numbers, using it from one to three days of work.
So if you are interested, again if you are interested,
there is an application until next week, or five until then more like four or five days left.
I would probably go to K-Pi link,
but you can go on the WhatsApp group.
There is a water group.
And here on the link, there should be a water link,
and you can join the group and go.
There is a link to the forum where you can participate in the group.
I would encourage you to do it because we do lots of hackathons.
Hackathons are those institutions on blockchain.
I mean, it is not only blockchain in general, but we try a lot.
We spoke with cool guys.
The first picture here is the one with Ethereum.
Here we did some workshops, we did some audios.
Here we participate, for example, on the global hackathon with one function crisis.
And if you are looking for virtual or master thesis, you can also end the blockchain.
Of course, I can help you with finding a topic.
Because Professor Prince is basically also part of the team and of the group,
so all the blockchain topics are easy to partner with the image.
This guy is supervising the virtual thesis.
He is now a startup here in Berlin.
This guy and I are also supervising to finish like two months ago.
He started at 3WDC after watching the console.
This guy started with a startup, raised lots of money.
This guy won a 6-digit hackathon here from Hacken, which he organized.
So it is a cool community to be in.
It is nice for you to find talented people who are in the blockchain.
So with that, I would say, we are 5.
If you don't have any questions, just write them down.
I can just find where the tickets you can see the circle.
By the way, do you have any questions?
Yes.
How did you find and report the guy that sold the...
Here I can tell you.
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
