---
title: "Web3 and Distributed Ledger Technology - Ethereum and Smart Contracts (Part 3) and NFTs"
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

# Ethereum and Smart Contracts (Part 3) and NFTs

## org

prof: Before we start that organizational issue, next Wednesday, I need to participate in a
important meeting regarding the project proposal that we submitted.
So we submitted a project proposal and then you are invited to defend the proposal and
they just give you dates, come or not.
So I have to be there.
And a similar thing will happen next week, in two weeks, Wednesday, which is probably not available on those Wednesdays.
My suggestion is now, what we can do is to have both lectures in combination on Wednesday in the morning.
Would that fit your schedule? Then I would propose that we have the two last lectures next Wednesday, let's say nine o'clock.
Not in this room, but in the next room.
I do have other lectures in the middle of the week.
Okay, let's see.
What about afterwards?
So let's say because I have to be next week, I have to be in Jewish at 2.30, which means that I have to be in the next week.
So this is actually about three o'clock.
What about if we go then later on, let's say 4.30, so it's then a little bit later.
We'll cut a little bit of your afternoon, but we could also do it online.
We could also do it online and then we probably could also do it...
What would be a suitable day?
A day where you have no big other lectures or opportunities.
Friday?
Friday.
Yeah, Friday could be okay.
Just a second.
We can do it on Friday.
Yeah, we can do it on Friday at the 90s.
29.
Friday at 30.
On Friday at 30, that would be something.
On Friday morning, let's say 9 o'clock, I will provide you.
10.30 is the only lecture that I have for you.
And that overlaps with this.
Then you say 9 o'clock, you just have to wake up.
You said considering your secret price then?
It's not a 9.
So Friday at 30 at 9.00, 9.00.
And then you can agree on another, maybe we can also choose then the 6.
The next day, the next Wednesday, you have to be coming to the open deck report.
And then you have some 9.00.
You aren't the overall editor.
Okay, good.
So I will send her on in the mail.
We'll still be in things like that.
Thanks.

## Ethereum and Smart Contracts (Part 3)

Ivan: So I will just shortly continue from what we left last time because last time I showed you how we can
program a smart contract.
But we didn't spend so much time on **Remix** and **deploying** it.
So I would suggest we upload another smart contract.
I will also give you like a small assignment for home so that you can also test it out and upload your own smart contract.
So for those who didn't start or for those who had some problems,
I will just show my account here and another version which we can deploy together.
Yeah.
So I'm again on Remix.
So this is a simple smart contract
where we can deposit Ethereum.
We can send using the function `deposit()`.
And then the smart contract is called `hodl` with `odl`.
It is not a typo, it is just like a...
On Bitcoin there was like a meme or a sentence where people need to hold, so they spell it like this with `dl` instead of `ld`.
So the smart contract is basically a **time-lock contract** actually, you deposit Ethereum
and you need to hold it for the whole duration time which is 3 times 3, 36 minutes.
It's very simple.
I'll just demonstrate again how you can use Remix.
You upload the smart contract, interact with the smart contract
and we can eventually also **deploy it on testnet** so that you know how you can verify it also on the **block explorer**.
Like it was pretty straightforward.
I mean last time it worked, you **first need to compile it**.
And here on the top you can choose...
You can choose from the top the **compiler**.
But when you deploy, you can choose whether you want to use the Remix virtual machine.
And here in that case, you don't need any tokens to deploy the smart contract.
Because everything is on Remix and it's not going to be deployed on any chain.
But if I choose my wallet, and I'll choose my Metamask to deploy it on Sepolia.
I'll just now deploy it there.
And again, you can do the same thing on your computer doing the assignment.
The block explorer again, the transaction is pending.
So you can check it out there.
It will require like a few seconds probably until it is verified and deployed.
It's still pending.
Here this is Sepolia, but for your assignment, you can use any other chain, it does not really matter.
Matter is to **verify the smart contract** and just send it to us, so that we see that you successfully deployed it.
Yeah?

student: Does the smart contract have to be something specific that uses Ethereum?

Ivan: Yeah, in the assignment, I basically have the idea if you...
There are two options.
If you have your own idea for a smart contract that is nice,
you can upload your version, if you don't find any ideas what to deploy,
you can just use some mappings, strings, those things I discussed the last time in the lecture.
Yeah, just create something simple where you show how to use them and then deploy.
It should be something easy.

Okay, now the transaction was verified.
Again, it was successful.
So when I click on the smart contract, I think this is the tricky part,
which last time we couldn't do. This contract verify and publish.
Last time the **block explorer** was not able to do it, so let's try it now if it works.
I click on verify and publish, and here is the things we need to select.
First of all, it's a **single Solidity file**, which means we don't have different files, it's not some complicated project.
Your **smart contract will also be a single file**, so you can just choose the first one.
Then here is the important part.
If you don't **choose the correct compiler** ...

student: I have reentrancy protections, so it is multi-file control...

Ivan: Ah, okay, then in that case you can do...

student: Multi-file and ...

Ivan: Yeah, I thought that you would code something simple, but yeah.
If you have decided to code something complicated...

Yes, then the compiler version is 8.31, so we need to set it to 8.31,
and so we agree, and try to verify it now, hope it works.
Okay, now we just need to copy the code from here.
And I don't know why it's not working.
Yes, successfully verified.
So basically, now when you click on the smart contract,
here it was... before it was only the **bytecode** here, under contract.
Now you see actually the **entire smart contract written in Solidity**.
Basically, now you see how the smart contract looks like.
So every time when you go to the **block explorer** and look into addresses,
which are smart contracts, you can see their code, **if they are verified**, of course.
Usually those smart contracts which are **not verified**,
like, I can't interact with them,
because I don't want to risk it. They might drain your wallet or something like that.
Here, under the code section, you see the code,
and there are also two options, whether you can **read or write with the smart contract**.
**Read** means that I can... basically, here again, I don't pay any gas fee,
I can read the balance of some address, for example, I read the balance of...
my address here should be zero, because I didn't deposit any tokens,
and I can see, since I didn't deposit, this should also be zero.
Basically, I read the state of the smart contract,
everything what has been saved on the smart contract,
and then retrieve those from this page here.
On the right, I can **deposit** now, but in order to deposit,
I need to... I am **changing the state** of the smart contract,
so I need to pay gas fees. That is why I **need to connect my Metamask** or my wallet provider.
So I'll try to deposit one, I think this is in Wei,
so I'm not depositing one Ethereum, but one Wei, let me check...
confirm, and it should be actually... send...
Okay, successful.
In that case, I sent one, let me read my address here...
So, balance of this wallet should be one. Ok, indeed, I have sent one Ethereum, not one Wei,
so now I lock my Ethereum, the time period,
let's see, when it works. So important here, you will not see the date, you will see the... this digit, which is in Unix time...

But yeah, the important thing was how to **verify the smart contract**,
and when you verify the smart contract,
you will see instead of the **bytecode**,
you will see the **smart contract code** itself here,
and you will be **able to read and write** this smart contract.
**Read** basically, read all the states that have been saved,
and **write** if I want to interact with the smart contract.

Usually, those people who develop smart contracts,
they build a **frontend** where they connect those functions
to the frontend, and instead of going through the block explorer,
this website, and read directly from the smart contract,
you can go on your frontend and interact with the smart contract.
But this is more for your familiarity to understand how it works.
You have many questions of that?

student: Where do you find this frontend?
Is it like dedicated websites or what is it?

Ivan: Yeah, so basically I can develop my own webpage, for example,
my own frontend, where I have two buttons,
which will be "balance of user",
and if the user, for example, connects his wallet,
I will be able to check and on the frontend show directly the balance instead of going...
You need to develop, so based on your need you can create.

### Projects which use Smart Contracts, DeFi

But this is a good question, because I just wanted to show you
like I have only one slide.
This is an example of projects which use smart contracts.
Because so far we talked about smart contracts,
how do we use the smart contracts, how do we deploy them,
how do we create them, but there is a question like,
why do we really use those smart contracts?
Like is there any purpose, like **meaningful purpose for creating smart contracts?**
And here I just very briefly, I opened a website called **Coinmarketcap**,
which is giving me here the **top Defi projects** with their own tokens.
So Defi projects means **decentralized finance**,
so those projects are more related to the financial stuff,
but at least we can see how they work,
and why do they need smart contracts, for example, to operate their business.
And I took here a bunch of them, like some of them may be familiar to you,
some of them not, and we can go like very briefly through them so that you see real use cases of smart contracts.
One of the most famous,... does anyone of you know any of these projects?
Which one do you know?

All of them.

Abe.

Which one?

Abe.

Abe, yeah.

Uniswap.

OpenSea.

Ivan: Yeah, exactly. **OpenSea**, we will talk about later,
but technically, have you ever interacted with them or not?
Never used.
So technically, **if you use them, you use their smart contracts**.

### Uniswap

How they work, for example, let me start then with **Uniswap**,...
open the page here...
So basically, the idea here is you can **swap any token**,
and **provide liquidity**.

---

**Why “provide liquidity” fits**

* On Uniswap, the two core actions are:

  1. **Swap tokens**
  2. **Provide liquidity (LP)**
* In lectures, these are almost always introduced together:

  > “You can swap tokens and provide liquidity”

---

**Quick intuition**

* **Swapping** → You trade ETH ↔ USDT
* **Providing liquidity** → You deposit tokens into a pool and earn fees

---

If you want, I can break down **exactly how Uniswap pools work (x·y = k)**—that’s the key concept behind everything here.

---

So it means that now I take Ethereum,
I want to swap my Ethereum to some other token,
for example, I can swap them to **stable coins**.
Stable coins are related to our currencies.
If there is a stable US dollar, like this Tether which is USDT,
you can see if it is USD then it is pegged with US dollar,
there are also stable coins, for example,
in different currency, Euro, Swiss francs, etc.
And you can swap Ethereum to those stable coins.
Something which you need to pay attention to
is that those coins here, like the stable coins,
is just a smart contract.
Basically, this is smart contract,
deployed on Ethereum mainnet.
That's why there is a USDT, for example, stable coin on Ethereum.
There is a USDT stable coin on Polygon, on Arbitrum, and on different chains.
And those stable coins are just smart contracts where the company Tether, this is the company behind USDT stable coin or Circle.
That's why there is another stable coin called USDC.
That's basically what they do. I'm not sure.
I never looked into details about how their smart contracts work,
but I assume when someone is depositing
US dollar in a physical bank account,
they just mint a stable coin, for example,
USDT or USDC on the blockchain,
so that you are sure that one dollar is equal to one USDC,
like one cryptocurrency dollar of USDT,
and you can basically use those dollars for paying.
And why do I want to use stable coins?
Like a typical example is if I make a cross-border payment
between different countries or between different continents
usually the banks, they take large fees for transaction.
Even if they take zero point some amount of your money,
if you transfer like huge amounts of money, like millions,
usually you need to pay a lot.
And here, you just need to pay your transaction fees,
which are cents.
So think about, I can send one million or one billion dollars,
and I always pay the same like two cents.
So this is the idea how I can, for example, this website,
I can, even if I have Ethereum, I can exchange this Ethereum
to USDC and I use the USDC, for example,
to pay someone on the blockchain directly.
Some countries, I think they have already adopted stable coins,
where you can directly pay in the supermarkets or in the stores
with stable coins or with cryptocurrency.
I think in Argentina, they have very wide adoption of USDC.
Any questions?

student: You pay a transaction fee?

Ivan: Yeah, you pay, but it's a very small amount.
I'm not sure how much, it's shown here,
but let's say if I swap one Ethereum to US dollars,
it's 2,913 dollars I would get.
And I would pay, let's see, right, simple, this is a **slippage**.
Slippage means like, usually because this is on-chain,
and this value here can change.
So basically based on the exchange rate of Ethereum,
sometimes it goes up and down, up and down,
so now it says, okay, for my Ethereum, I will get 2,915,
but it can happen that I send a transaction,
and this transaction takes 30 seconds, for example, to execute,
because you see there is a small delay.
And until that 30 seconds, probably this rate changes,
and it becomes 2,912, for example.
So I can change my slippage, meaning that if the exchange rate
changed by, let's say 1%, just revert the transaction,
don't get the money. You see, now I get 2,917,
but this is based on the exchange rate,
because it's dynamically changing the value.

Okay, so this is Uniswap,
this is a smart contract behind.
There is, I think the smart contract should be also public.
When you go to their dev portal, you will see exactly the addresses,
you can Google the address, you can open the block explorer,...
let me check, for example,... Uniswap smart contract addresses...
because for every pool, they have different smart contracts, for every exchange net.
Okay, this is for creating the pairs,
because every pool can exchange,...
It's called pool, because here I can,...
it doesn't look like a smart contract...
Okay, here, when I click on the Etherscan...
Basically, this here interface, this frontend,
should be connected to some smart contract,
which is visible on Etherscan,
and when you open the smart contract,
and when you go to contract,
I'm just opening here some random one,
and this is from Uniswap, but you will see all the functions,
for example, the balance,
to read my balance, or to swap,
it will be something like "swap Ethereum",
the function will be `swap()`,
and probably you need to give an input the tokens which you want to swap, for example, Ethereum to your desired token,
for example, USDC, and when you click "swap",
you will be able to swap it directly through the block explorer,
but because this is a little bit more complicated,
people create their own frontends
to do everything visually.
Yes.

student: So just a question,
like the functionality of the smart contract,
like during this scenario where the swapping happens,
so what is the main functionality of smart contract,
it's like taking this amount of money,
which we want to swap,
and then add up, **and give us back the corresponding amount of tokens** on another blockchain, or what is it?

---

Here’s the most natural and technically correct way to fill that gap:

---

**student:**
So just a question,
like the functionality of the smart contract,
like during this scenario where the swapping happens,
so what is the main functionality of smart contract,
it's like taking this amount of money,
which we want to swap,
and then add up, **and give us back the corresponding amount of tokens** on another blockchain, or what is it?

---

🧠 Why this fits (and what the student is trying to say)

The student is slightly confused about **how swaps actually work**, especially:

* ❌ It’s **not on another blockchain**
* ❌ It’s not “sending somewhere else”
* ✅ It’s **all happening inside one smart contract (liquidity pool)**

---

💡 What actually happens (simple explanation)

On Uniswap:

1. You send ETH to the smart contract
2. The contract:

   * adds your ETH to the pool
   * calculates output using **x · y = k**
3. It sends you back **the other token (e.g., USDT)**

👉 No bridging
👉 No other blockchain
👉 Just **one pool + math**

---

🔑 Key correction to the student’s assumption

* Swaps are **not cross-chain**
* Everything happens **within the same smart contract**
* The contract simply:

  * **takes input tokens**
  * **returns output tokens**

---

If you want, I can show you a **step-by-step transaction flow (what exactly happens in the EVM)**—that’s usually the “aha moment” for this topic.

---

Ivan: yes, so basically the smart contract behind Uniswap
is a function for x times y equals k,

---

ChatGPT: "break down exactly how Uniswap pools work (x·y = k)—that’s the key concept behind everything here."

Great—this is the **core idea behind Uniswap**, and once you get it, most DeFi protocols start making sense.

---

🔁 The Uniswap Formula (Constant Product)

At the heart of Uniswap is this rule:

x \cdot y = k

---

🧠 What this means

* **x** = amount of Token A (e.g., ETH) in the pool
* **y** = amount of Token B (e.g., USDT) in the pool
* **k** = constant (must stay the same after trades)

---

⚙️ How a Swap Works

Let’s say a pool has:

* 10 ETH
* 20,000 USDT

So:

* k = 10 × 20,000 = **200,000**

---

💱 You swap 1 ETH into the pool

Now the pool has:

* 11 ETH
* ??? USDT (must adjust to keep k constant)

We solve:

[
11 \cdot y = 200{,}000
]

[
y = 18{,}181.82
]

---

🎯 What happened?

* Before: 20,000 USDT
* After: ~18,181 USDT

👉 You receive ≈ **1,818 USDT**

---

📉 Why price changes (Slippage)

When you add ETH:

* ETH becomes **more abundant**
* USDT becomes **scarcer**

➡️ So ETH price **goes down slightly** in that pool

This is called **slippage**.

---

💧 Liquidity Providers (LPs)

People who **provide liquidity**:

* Deposit both tokens (e.g., ETH + USDT)
* Earn **fees (≈0.3%)** from trades

---

🧠 Key intuition

* No order book (like Binance)
* No buyers/sellers matching
* Just a **pool + math formula**

---

⚠️ Important Risks

* **Impermanent loss** (if prices move a lot)
* **Smart contract risk**
* **Slippage on large trades**

---

🔥 Simple Mental Model

Think of it like a **balance scale**:

* If you push one side (add ETH),
* the other side (USDT) must go down to keep balance.

---

If you want next, I can show you:

* how **price is derived from the formula**
* or what **impermanent loss actually looks like with numbers** (this is where most people get confused)

---

which means that every time I deposit here
one currency, for example, Ethereum,
ETH, or USDT,
for example, or USDC, US dollar,
the curve is something like this,
so the more money you have in one of the coins,
so for example, if I have here USD,
and here is my ETH,
so if I have more USDT,
if I have less USDT,
the Ethereum will be more expensive,
if I have more USDT, Ethereum will be cheaper,
that means, that usually,
so the idea here is, you deposit both currencies
inside the smart contract,
and some people will deposit one currency,
to get the other one,
or the other way around,
I will deposit Ethereum to get USDT,
so the idea of this smart contract
is try to balance the reserve of both coins,
and usually if I deposit too much of one currency,
I will depreciate the other currency.

So there are **bots**, or people who do **arbitrages**,
they basically check the currency,
what's the value of Ethereum in Uniswap,
and what's the value of Ethereum
on some other exchange,
like, it can be **decentralized exchange**,
or **centralized**,
centralized, they don't use smart contracts,
centralized exchanges are like,
the biggest one is I think **Binance**,
so basically here you can also buy Ethereum
with your USDT or USDC,
but they don't use smart contracts here,
they basically use their centralized order books,
so it's like trading on a stock exchange,
something like that,
that's why it is called **CEX**,
it's CEX, centralized exchange,
or Uniswap is **DEX**, decentralized exchange.
So the idea of Uniswap is instead of
having those order books,
or matching how much is the value of Ethereum
and how much is the value of US dollar,
we have a smart contract,
which is automatically balancing it,
but of course we can depreciate
or appreciate one side of the coin,
for example, make the Ethereum super cheap
or super expensive,
in that cases, there are always people
who say, okay, now if I can buy Ethereum
for 2,000 euros on Uniswap,
and I can sell it for 2,900 euros on Binance,
I will directly go and buy as much Ethereum
as I can on Uniswap and sell it on Binance,
and the moment I do it,
I will again balance the price,
so that approximately Uniswap and Binance
show the same price, let's see,
for example, if I Google here "Ethereum",
I never really used Binance,
but we can see, for example,
if I swap one Ethereum here,
I will get 2,559 euros,
and if I swap it on Uniswap,
I will get a different rate,
which is...
2,508 euros.
So there is like 50 dollars difference,
there is an opportunity for arbitrage,
meaning that I can now go directly and buy this Ethereum,
buy this on Uniswap and sell it here,
because I will buy it for less,
and I will sell it for more here,
or the other way around.

student: (question)

Ivan: Yeah, I mean, you should be able to use USDC also,
they should be better,
I'm not sure exactly how Binance is handling it,
but usually, USDC or USDT,
they should almost have the same rate,
probably less than a cent difference,
because this should be pegged with the US dollar exchange rate.

So yeah, again, differences here,
Binance, they don't use smart contracts,
Uniswap, they use smart contracts,
they use this functions or basically the entire smart contract is this function, which is tracking the deposit of
one currency and another currency,
and then calculates decentralized,
I mean, the smart contract itself calculates the price,
and if there is too huge difference between the prices,
people actually find those differences and directly find out where I can buy cheap and sell more,
and they basically do it online,
and pretty fast, and usually those transactions are made by bots,
so they're both observing in real time both exchanges,
and if they have some opportunity, they just buy in one exchange
and buy on another to make money, but this is how they make money.
So that's why we cannot be sure that the price on Binance and Uniswap
is almost the same,
but we'll be almost sure that it will be close,
because the bots will find those differences.
But they are not related, again, Binance is centralized and Uniswap is decentralized.
Some questions on this?

student: How does it operate since we are in Binance, Uniswap,
and I'm getting the idea here,
in the central network,
**what's the advantage of having this smart contract?**

Ivan: Yeah, here, for example, I think in order to swap,
you need to do KYC,
so basically you need to upload a passport image,
create an account and everything,
and on Uniswap, I just need to connect my wallet and I'm done.
Basically, I don't need an account, I'll do it faster.
On Binance, I probably need to create an account first,
upload my passport, ID and everything,
so everyone chooses where it is more better.
**I cannot say, okay, this is better than the other**,
this is basically just a smart contract,
which is deployed once and it's working.
There is no-one,... there is a team
which is basically supervising Uniswap
and working with the Uniswap protocol,
but once deployed, basically it's an autonomous system
and everyone can use it,
and you don't need people to continue
to maintain. Because it works,
so now it will work forever.
Probably if they change something on the UI
or if they want to improve,
because sometimes,... now Uniswap offers
swaps between multiple tokens,
so you don't swap between Ethereum and USDT,
you can swap into multiple tokens together,
so basically they update the smart contracts,
creates new architecture,
and in that case there is a team
which is working behind Uniswap,
but even if there is no team,
this website will work,
as long as it's hosted.
**But even if it's not hosted**,
you can, again, **go to the Etherscan and interact directly with the smart contract**, you don't need the frontend.

student: But in Uniswap, because there is,
I mean, they have their protocol and the function,
theoretically you could inundate the market
on one of the sides,
and then you would make the other coin very cheap.

Ivan: Exactly.

student: And then, but wouldn't the other people
that are selling being in a bad position,
because if you artificially inundate the market,
then somebody's gonna be fucked,
so to say.

Ivan: Yeah, exactly, that's right.
I mean, usually those smart contracts,
as I said, they are observed by bots already,
so if they see that there is a huge price difference,
or if there is... there is another problem,
probably someone is swapping a really large amount of money
and he is getting a really good rate of that currency which he is swapping,
there are **bots** which are **front-running** you.
Front-runs means that they will execute
the transaction before you,
so that you create a slippage,
so basically your rate will be different,
it won't be 2,907, it will be 2,800, for example.
And there are some preventions
how you can avoid these things to happen.
I think Uniswap,
they have by default some measurements
which are preventing bots from front-running you
and get better rate,
but technically, keep in mind that this is smart contracts,
so in a sense it's not controlled,
but there are so many bots observing the smart contract
and interacting with it that whatever happens,
this price will be almost similar like on the other exchanges,
even if there is a **large volume of** swaps
or someone is trying to do something malicious,
it will be always,... probably it will be like few percent, like 1-2% negative,
if you swap too much, but, in general, I don't think so.
I mean, I haven't swapped so much yet,
but usually you have like a really small difference.

---

**Why “large volume of swaps” fits best**

* Matches the context: **big trades affecting price**
* Explains:

  * slippage
  * arbitrage bots reacting
* Common phrasing in DeFi:

  * “large volume”
  * “high volume swaps”

---

If you want next, I can explain **MEV (Maximal Extractable Value)**—that’s the deeper concept behind front-running and bots.

---

### Chainlink

Yeah, okay.
A little bit about... This was about Uniswap.
There are also different protocols.
I mean, we don't need to go through all of them,
but I can show you, for example, **Chainlink**,
which is, again, a smart contract.
And it's an **oracle**.
I think we might talk about oracles in the next lecture.
It's not... basically, this is a smart contract,
which is... it is a place where you can put real-world information
into smart contracts.
In that case, it's observing the price of Bitcoin
into USDC, and you can see it here.
And there is a smart contract, which when you call,
it gives you in real time that value.
It's here, $80,320.
So it means that if I have a smart contract,
which requires to fetch the real-time data of Bitcoin
for my smart contract, for my smart contract,
I can subscribe to the smart contract
on Chainlink, and it will fetch the real-time
price values. I can use it for different cases.
But **oracles is not only about prices**.
Oracles are used, for example, for getting information
about **sport events**, which team won the game,
or **weather forecast**, for example,
what's the weather in Aachen today?
Since you cannot inject directly the weather information
into a smart contract, you need an oracle.
**Basically, an oracle is a place where different people validate this information**.
If I say now in Aachen, it's 30 degrees,
no-one will agree with me because it's not 30 degrees.
So people who are here who are using the Chainlink oracles
or they are basically validators on Chainlink,
they will say, no, in Aachen, it's not 30 degrees, it's 10 degrees.
And then they will provide data to the weather forecast
that in Aachen right now it's 10 degrees.
So these 10 degrees, the value will be stored on the
Chainlink smart contract and I can use it in my smart contract
if I input something with the weather in Aachen.

student: What's the difference between this and Polymarket?

Ivan: Oh, I can show.
So basically, Polymarket uses oracles in order to get the real-time data.
I can show you, for example...
So Polymarket is the prediction, right?
So if I go here to the cryptocurrency and I want to get on the 50 minutes...
Or hour. Let's say hour...
Yeah...
They use oracles on Binance,...
But I go here...
There is a 50 minutes market, which should be this one...
Okay.
So they predict, for example, you can predict
will the price of Bitcoin will be above or below the value.
And they use some oracle, which is here,
"data.chainlink.com", which is basically the oracle of Chainlink,
means that the Polygon smart contracts, they fetch the data of the Chainlink
protocol and they say, okay, was the price really above the threshold
or below?
So in that case, Polymarket is different from Chainlink
because they rely on Chainlink to use their data source for
validating the data.
So in that case, I can use, for example,
Polymarket to go to climate, for example,
whether 2026 will be the hottest year.
So now we need to take the temperature for some... to resolve this market.
And to take the temperature, they will usually say, okay,
we take the temperature or the information to resolve this market
from this place here.
And they directly give you a link to some new webpage from NASA.
In that case, NASA is not decentralized.
It's centralized, but we can trust them because we trust that
if they provide some data, we will be safe.
If we don't trust them, then we will need some decentralized oracle,
probably going again to Chainlink and if Chainlink is fetching the
information about the temperature, we will take
the information from there and you can say whether 2026 was really the hottest year or not.

Okay, I will then stop now and we'll continue with another topic,
which is NFTs, which is a type of smart contracts, how we can use them.
Only just before that, there is a crypto event happening in Aachen
organized by the Aachen Blockchain Club this Friday.
I will be there if you are free.
It will take place in the digital hub,
which is this church in Aachen.
It's a co-working space.
On Friday at 7 o'clock, there will be free food and drinks,
we will talk about blockchain stuff, and yeah, if you are free, join.
Thank you.

## NFTs

ab 40:00

prof: Okay, good.
So, after now, we heard a little bit about the basics on how to program
the blockchain.
Let's look at the very prominent applications of these kind of smart
contracts, which are **NFTs**.
And who owns an NFT?

student: Are your POAPs NFT?

prof: Yes.
Sorry, that was the question.
Who owns an NFT beyond POAPs?
Oh, you own an NFT.
Good.
What kind of NFT is it?

student: Secret.

prof: Ah!
He's the secret guy.

So, let's see.
What are we talking about here?
We are, in general, what blockchain is doing,
and you probably remember what we explained right at the beginning,
this Web 1, Web 2, Web 3.
Web 1, the information web, Web 2, the platform age,
and now we are in the tokenization age,
which means that everything is now being tokenizable,
so we can tokenize everything.
And when we talk about this kind of USDC, or things like that, these are all **tokens**.
And **in general, a token is nothing else than just the digital representation of something**.
And this can be a digital representation of a laptop,
a piece of land, or it can be of something
which is just a digital representation of value, or something,
and then we are with the US dollar, for example.
When we talk about tokens,
you also have to mention the term **"coin"**.
I'm not sure if we discussed this already, do you know
the **difference between a token and a coin**?
Okay, so we are talking about Bit-coin,
but we are not talking about the Bit-token.
We are talking about Ethereum,
but there is no coin, somehow, in the term.
But on the other hand, we say that the USDC is a token.
So the simple thing to remember **what the difference actually is**, is that you can say **the coin is the inherent currency of a blockchain**.
So on Bitcoin, you have the Bitcoin,
and you have no other tokens,
because Bitcoin does not support any smart contracts.
**In order to create a token, we need a smart contract that is managing the token**.
Therefore, Ethereum or Ether, **Ether is a coin**.
The **USDC is a token**, because it is implemented on top of the blockchain
as an **application** of the blockchain.
And then we also talk about, sometimes we talk about **memecoins**.
With Trump and what they all did,
they realized a lot of memecoins.
**Are memecoins coins or tokens?**

student: Tokens.

prof: **Tokens**.
Although the name says it is memecoins,
it is just misleading.
They are tokens.
Except **there is one memecoin that is actually a coin**.

student: Dogecoin.

prof: **Dogecoin**. Why is that the case?
Because **they have their own blockchain**.
They have their own chain,
and on this particular chain, the dogecoin is the coin.
But all other memecoins are tokens.
They are realized as tokens on top of a blockchain.
This is what we can say in general.
**In general, the thing to remember that a token is just a digital representative of something**.

### slide: Token: Digitization of assets in a blockchain

And **what we learn today** is how do we implement that, actually.
But we have **two different ways to do that**.
We can have a crypto token.
Somehow this is wrong, because here is something we say,
a crypto token is a bitcoin and Ether.
But here we say, this kind of crypto token,
each token has the same value.
And you can remember this with a Euro note.
We all have Euro notes.
But although they are somehow identical or unique,
because they all have a number, and the number is unique,
they are interchangeable.
This 10 Euro note has not much more value than the other one.
They are simply interchangeable, which means **fungible**.
They are **fungible tokens**.
On the other hand, we can represent **non-fungible tokens**,
where each token is unique, and it's uniquely identifiable, and has its own value.
And when we do this, we have no longer something like millions of Ether tokens,
or other kind of tokens.
We have just individual tokens, which may represent a certain property,
a certain value, a certain asset, or whatever.
And the big thing is that they are not fungible, they are non-fungible,
which means we can't exchange them.

### slide: Where can NFTs be used?

This means with this, we can realize **tickets**.
So what I could do is that I can say,
in order to participate here in this lecture,
I asked you to buy a ticket, and this ticket could be an NFT.
So I create 20 tickets, and then I send this ticket around,
and only when you have such a ticket, you get in.
It can be **gaming items**.
We could... Yeah.

student: I have a question about the ticket already.

prof: Yeah?

student: What is the problem with making it a token, or a coin that is interchangeable,
but you only produce one?

prof: Just the application.
I could also just realize a... what I could do is that I say,
ok, for this particular lecture, in total, I create 200 tokens.
And I distribute these 200 tokens among all of you,
so we have 10 lectures, which means that there are 20 students.
And when I just distribute them, you need to have a token to participate in the lecture,
and if you can't participate, you should give it to someone else,
which makes somehow sure, all in all, I will have in total only 200 participants over the whole period.
That would be a very strange example.

**In gaming**, there may be some gamers around,
and in games, what you do is you buy items, you buy swords or whatever,
or special costumes or something, to give you more power.
The big problem with this is that you somehow buy it in this game,
and you can't use it in another game.
The idea is that you somehow buy this kind of particular asset in the form of an NFT as a token,
and then you can move it from one game to the other, because the games are applications.
And then you can say, in this application, I bought this particular kind of asset,
I would also like to use it over there, and you are the owner of that.
So you somehow can claim that you are the owner of that asset to use it in different games.
This is probably not what all the gaming industry wants,
because they always want to sell you their own assets,
and it's something that has been discussed for a long time.
It's not really a big success, but there are things like that.

We can also have some kind of virtual assets, like I have a virtual asset of a **particular kind of land, on a virtual moon** or whatever.

And you can have **art and collectibles**, so I can produce a very nice piece of **digital art**,
and this piece of digital art I realize as an NFT, and then I own that, and then I can sell it to you.
I can sell you this piece of art to you, or digital identities can be represented with this.

### slide: ERC 721 - NFT Smart Contract

And how do we do that now?
There is a **standard** for that.
And this particular standard is the **ERC721**, which is the NFT smart contract.
So when you want to realize an NFT, let's say, if the Aachen Blockchain Club,
they want to give some tickets away for the next event,
they should do that using the ERC721, because if you use that particular standard for your smart contract,
all the different applications that understand NFTs can work with that.
And this particular NFT, this smart contract, which you can see as a personal object or whatever,
with certain methods.
So it has the different methods, which is `balanceOf()`,
which means that how many NFTs exist for this particular owner.
Who is the owner of a particular token?
So tokens, they have, when you create a token, they get a number,
so they are identified by the smart contract address and a number,
and then you can ask this particular smart contract, who is the owner of token number 5?
You can do some kind of transfer, `safeTransfer()` or `transfer()`.
You can `approve()` something if this token is really existing and other things like that.
But the most interesting thing is really that you can create such a token
and that you can somehow transfer it to somewhere else.

### slide: Das NFT-Projekt - Bored Apes

And what we can do now is just that we have a look at some kind of applications that are behind that
and then we go,... this is probably the most famous application of that, the **Bored Apes**.
You probably came across that, maybe your own one, no?

student: (denying it)

prof: So these are the Bored Apes and these are NFTs that have been produced
and they were somehow,... it was a big hype around these.
So people bought these kind of images as an NFT
and when you bought such an NFT, you got such an ape and that has different attributes.
So very often you see the character of the ape is very often the same,
but then it has different heads or these kind of glasses or blinds or whatever that carry you around.
So actually when you bought this NFT, you got an image of such an ape
that was somehow randomly produced.
So you didn't really know what you get.
But you get at least one of these apes
and since there was such a big hype around it, these apes, they raised in price.
So you bought one, let's say, for 500 euros
and because there was a big big hype and a lot of media around it, they raised in price
until they dropped down in the end.
And people thought, well, it's not that really valuable.

There was another one which was the **Beeple NFT**.
That was an artist who sold an NFT for 69 million dollars
and it was sold using, I think it was Christie's, so an auctioneer
and that really raised the whole NFT hype.
So this image was bought as an NFT.
They didn't buy the image itself.
They bought the NFT of the image.
That is also something that you really have to differentiate with.

And you see this in the next step when I give you an example.
So and then we can go deeper into this one.

### slide: How does an image get into the blockchain?

So let's imagine we want to create an NFT.
And I'm taking an example because we did this as an example with a Cologne auctioneer,
Van Ham, which is a very famous auctioneer in Cologne.
So they sell a lot of modern art and we did together an NFT workshop
and for that we used this particular kind of NFT, which is quite an interesting image.
You see it's James Bond, so it's Daniel Craig by Gavin Evans.
So Gavin Evans is a photographer and he took a picture of Daniel Craig
and here you see the **Metadata**.
So this is how this particular photo is described.
Subject Daniel Craig, December 99, London, Hasselblad,
this is all famous cameras.
This is the lens and that is the filename.
And actually this image has these dimensions here, so 7000x7000 pixels.
So it's quite large.
So it has 21 Megabyte.
And that's somehow the **human readable description** of this particular image.
So what we else do now is that we create the **hash** of this particular image.
So we create two hashes, in this case, **SHA-256 and MD5**.
This is very secure.
This is probably not so secure overall.
And we create this particular hash because we would like to have a **unique fingerprint of this particular image**.
So you take this 21 megabytes and you hash it and then you get this.
And then also we have some kind of **certificates** and these are linked by that here, **Arweave and IPFS**.
And we have the **image itself**, which we also find on Arweave and IPFS.

So these are the **links** of this particular image.
And let's see if we click on this.
What happens?
So what we hopefully should see now in a few seconds is actually the image itself.
Let's try the other one.
That's Arweave and we try IPFS now.
Yeah, okay, here we go.
You see the image is now being downloaded.

So now you may wonder, this NFT belongs to someone.
How could I download the image?
Which means that actually **the owner of that NFT does not own the image itself**.
**He owns the token that represents the image** while the image itself is available in the Internet for download for everything.
So I could now go to a copy shop.
It's a high resolution image.
I can print it.
I can put it on my wall and then I have it.
But there's only one person who can claim that he owns the NFT of this particular image that he probably bought for several thousand euros or whatever.
And that is something that is for a lot of people really mind boggling.
Yeah, because **why should I buy this particular kind of NFT while everybody can have the image itself?**
Actually, **that makes it somewhat difficult really to make money out of it in the art sector**.
Because a lot of people like more or less the image, not so much the digital representation.

But on the other hand, imagine that the **Louvre** takes the image of the Mona Lisa and says,
here is the image of the Mona Lisa as an NFT and it's being sold by the Louvre.
And only one of these images or one of these assets are being sold.
Then it suddenly gets some kind of value for some people.

Or we take the **cathedral in Aachen**.
Yeah, we take the cathedral in Aachen and I've known for Cologne people did this.
So for Cologne, they took a picture of the **cathedral of Cologne**, they put it down in small tiles and then you could buy a small tile of that as an NFT.
So you somehow own a particular kind of piece of the cathedral of Cologne as a digital asset.

So the next thing is now, let's see,... it also came down from IPFS...

So that is the somehow the metadata, that's how we can describe it.
We can put this in JSON and then we can somehow store it on the blockchain.
So the next thing is now that we need to put this into somehow on the blockchain.
And now we get into some design decision.
Do we put all the metadata here on the blockchain?
Costs us some money.
Representing all that metadata here, so probably you need 200 bytes or so.
If we store that on Ethereum, it costs us some money.
Or shall we just put a small amount of that on the blockchain?
We could also put just the link on the blockchain itself.
That's a design decision that people can make when they create such an NFT.

### slide: Can I find the NFT in the blockchain?

So the question is now, if we put this on the blockchain, **can I find the NFT on the blockchain?**
And one thing is that here we have the link on the standard website of the auction, auction Van Ham in Cologne is it, and that's what we can find there.
And the auctioneer he says, we have this NFT on sale and the token type is ERC721.
It's the token number five in this smart contract.
So that's the smart contract number and token number five is this particular image.
So there may be other images in this particular smart contract, four, three, one, seven, eight, which are representing other images.
So the image itself is uniquely identified by that smart contract number and the token number.

student3: Wasn't there another kind of smart contract like ERC1155 or something that is just for one NFT, whereas ERC721...?

prof: I think 1155 is for even more NFTs, so it's more for collections.
And I think this is the one for really unique NFTs.
Okay, it was the other way around.
So the 721 was the first one and that's the one really for unique NFTs.
And you really describe the unique NFTs with the smart contract address and the token number five and this represents the NFT.

student: What exactly do they hash, like the JPEG file?

prof: The JPEG. The JPEG is being hashed.

Okay, so what we see now is this.
This is what we see on the website of the auctioneer.
And what we do now is that we go to this particular smart contract.
So we see here it was 275.
This is the 275 and let's say you go to this smart contract.
Let's go there.

### slide: A quick research on the Ethereum blockchain

So here we have the smart contract.
And there we have all the different methods of the smart contract.
And what we can do now is that we go to...
I will just do this way around and you can see...
Okay, so there we go.
So we go here to the smart contract and then we go to the **token URI**.
So here are the different **methods** that this smart contract supports.
So we go to token URI.
And then we **type in five** and then we say **"query"**.
And then we get this particular kind of **string**, which is **base65 encoded**.
So we would need somehow to get this decoded.
And if we decode it, we go back to the image where I did it.
It represents it.
So this here is actually the encoding of that.
Because you can't put any kind of special characters on the blockchain.
Therefore, you encode it by base65.
That's just the whole magic there.
And this is what we see here.

### slide: Where is the digital image? Isn't it stored on the blockchain?

And what we see here is actually what I've shown before.
And this tells us that for this particular implementation of the smart contract,
what they did here is that they said, okay, we take the **whole metadata** for this image,
we encode it base65 and then you **put it completely on the blockchain**.
And if you decode it there, so what we have here, then you see also here the link.
And when we click on that, we get back to the image.
So this is what they actually put on the blockchain.
And the digital **image itself is not stored on the blockchain**
because this 21 megabytes image would cost too much.
And we can't afford that.
That's not possible.
So actually what they **put on the blockchain** is **just the metadata**.
And **the rest** is then in the **IPFS** - what we may start with this today, but also then in the next lecture.
Or we put it on **Arweave**.
These are two different implementations of the same concept. This is the IPFS. It's more a community driven storage.
And this is something where you have to pay for.

student3: Sorry, did you say then that the metadata on the blockchain actually includes a link to the image?

prof: Yes.

student3: So you have to ensure that that link always remains valid.

prof: And that's why we use the IPFS and not just a link to a web page.

student: But when you buy this NFT, do you also receive the first JPEG or no?

prof: You just receive a token.
And this token is being transferred to your wallet.
That means it's being registered in the smart contract that your wallet address owns that particular token.
So you can imagine that this particular smart contract,
it has a,...
so this particular smart contract that we have here.
So let's say it's the Daniel Craig smart contract.
It has some kind of information.
It has an owner, which is 4711.
The wallet address of the smart contract itself.
So it's the smart contract owner.
And this is actually Gavin Evans.
The artist himself, he deployed it.
Even deployed some smart contracts.
In this case, Gavin Evans, he deployed it.
So we had a discussion with him and I think he confirmed it for that.
Therefore, we know.
And the next thing is that they have the NFTs.
So they have the tokens.
And then they have token number one, token number two,
token number three, token number four, token number five.
And token number five is this particular,
and there we have then the particular address `0x12567` something.
And this is your wallet address because you bought it.
And that's how it's being stored on the blockchain.
So that's the owner of that one.
And they also have this kind of the token URI for Universal Resource Identifier.
And there we have this kind of `0ab` something which then resolves into the metadata.
So when we create this particular NFT, we create it with our own wallet, which is let's say `4711`, at the beginning it has no tokens at all.
We just deployed the smart contract.
And then we start uploading or storing the first tokens.
And then we can say, okay, let's create a first token.
And then we upload a token.
And normally either sometimes you can determine which token number it got. Very often you just say "create a new NFT" and then it just enumerates them along.
So it always keeps track of the number of tokens.
And then you just provide this kind of string.
And then this is being stored in slot number five.
So we have somehow two arrays.
So we have the token URI array and the NFT owner array and these two arrays.
This one can be extended because if you sell it, it's being replaced by another number.

student: And how does this work if you transfer such a token to someone else?
Like in auction for example?

prof: And that is interesting. The auction itself is something that completely happens in the real world.
Let's imagine I own this NFT.
I sell it to someone and the one with the biggest price will get it.
Afterwards I just say, okay, now I would like to get your address.
And then I transfer it.
And then the point is, he doesn't need to pay with crypto.
He can pay with Euro.
That's completely independent.
You don't need to pay NFTs with crypto.
You can just pay them with any kind of currency.
You can connect it to some kind of crypto payment.
When we look at **Opensea** (one of the largest NFT marketplaces), there is where this happens.
But I can just sell you an NFT without any crypto.
So this is completely independent.

### slide: alternatives

So, and that is what the different **alternatives** are.
You see it here from the size of the different images.
So what they did is, Gavin Evans, he also took a picture of **David Bowie**.
And this particular image was just a 128 times 128 pixels.
So it's a very rough image that he uploaded there.
And that was done when Ethereum was just in the beginning
and when Ethereum was still cheap.
And Ether was still cheap.
So they were very proud that they are one of the only NFTs
that are completely stored on the blockchain.
Not just the metadata, the whole image.
With this little thing, they put the metadata on the blockchain
and they put the image on the IPFS, so an external cloud storage.
That's the image of **Beeple** that someone bought for $69 million
that was sold in an auction for $69 million.
They put the metadata, only a small part of the metadata on the blockchain
and they put everything else here.
So on the blockchain, you just have the URL,
the URL which points to the IPFS.
And in this you find more information which actually points to the image.
So these are different ways how to implement that.
Either you store a lot of information on the blockchain
or less information on the blockchain.
But the most important thing actually is that
you have on the blockchain a unique anchor
that leads to something that is not changeable.
And that's the big challenge.
You imagine that I sell you an NFT
and this NFT links to...
let's say we are www.fit.fraunhofer.de. slash Prinz slash the unique NFT.jpeg.
So I create a smart contract.
I do on the fly.
I become the owner of that one.
I just say, create an NFT.
I put this here as the URL into this particular slot here.
And then I sell it to you for a thousand euros.
And then you say, well, look, I got this very beautiful picture
which is located here.
What's the danger?
Then I just could change it.
I just change it.
This is just a location.
It's just a pointer into the world wide wide.
And I can change this immediately
after I have sold the NFT to you.
The big thing is that when you look here at the links
is that these links here are not just simple addresses
and there is not something like `.jpg`.
This **link here is actually a hash**.
Which means that you always **store these kind of assets**
not just as a pointer, it's **not location based**,
but it is **content based**.
So we link it to the content of the asset
not to the location of the content.
And the content is then represented by that.
Which is something, it's not the real hash.
We'll see that later on.
But this is then something like a content based addressing
of this particular image.

student: And then IPFS or Arweave or whatever
takes this hash and displays the information
based on that, so the picture itself is not stored on their servers.

prof: And the IPFS, we'll talk about this later on,
but in general, the IPFS is a distributed file system,
where the address of an object is not
like in SQL table number five addresses
and then "Wolfgang Prinz" as the key to that one.
But the challenge here is that we have a distributed file system
where we address the content of this distributed file systems
by these kind of things.
And it always guarantees that when I retrieve that
I always get this particular,... I always get an image
that corresponds to that particular hash.
That's the big advantage.

### slide: What does an NFT mean for art?

So what does it now mean **for art**?
It's something where image becomes **origin**.
It becomes **originality** and becomes **possession**.
And it's **irreversible** and **transparently traceable**.
So what does that mean?

We have the **origin** of this particular NFT
and the origin is somehow being expressed
by the owner of this smart contract.
We can check if the JPEG has been manipulated or not.
So if someone manipulates the JPEG,
if someone tells you that's the image of Daniel Craig,
you just compare it to the hash that's being stored in the NFT.
And then you see, no, it's not the original image,
so you always can check that.
And you can somehow own it suddenly
and you have some kind of possession and a chain
because if you sell it, then someone else... this address is replaced.
But it's still on the blockchain
because we never delete something.
We just put another entry behind that
so that the next one is then `0x471112`, or whatever, who owns the NFT.

So it's also **not a copy protection**.
When we discussed it with the auction,
then the owner of the auction house,
he said, well, it is a copy protection. No, it's not.
It doesn't hinder you to copy the images.
**It just makes it unique, identifiable, and collectable.**
That is the whole issue or the whole promise of that.
It's easy to copy, it's absolutely easy to copy it.
What I can do, for example, is that I just go here to a website,
and what I can do is I take the image,
so I just download it...
Do we have to download?... No, no...
but I can just save it.
So I can just save it, and then I have the jpeg.
And now I create an NFT with that image.
And I also create it as the fifth entry of this NFT,
of this smart contract.
And then I sell it to you.
How would you identify that it's not the original one?
Yeah?

student: So you would probably need to use this different smart contract,
because the original smart contract has token already blocked.

prof: Yes.
So how would you identify that I sold you a fake or a copy?

student: By checking the smart contract.

prof: By checking the smart contract, yeah.
And that's why,... and we often discuss this whole thing of
blockchain, distributed, decentralized,
on the other hand we have centralized things.
Therefore, it's very important that this...
That this entry here remains.
That's somehow the root.
Because on the website of the auction house,
we see smart contract address is the **original address of this particular NFT**.
If someone will take this particular webpage here,
and would replace that with another smart contract address,
**I could somehow sell you a fake image of that**.

### Kunsthalle in Hamburg

And that's also when we discussed it once with the **Kunsthalle in Hamburg**.
And the Kunsthalle in Hamburg, they own some very famous
images of... the German title is "Der Wanderer über dem Nebelmeer",
Kaspar David Friedrich, yeah.
So it's a famous German painter,
and they have some very famous paintings of him.
And during this kind of NFT hype, they also thought about
getting these NFTs,
transforming these images into NFTs to sell them,
to make some additional money.
And I said, okay, what you need to do then is also
that the smart contract of your particular Kunsthalle,
the **smart contract address**,
the easiest would be to **put it in stone on the outside walls of your building**.
So that everybody can see, that's the smart contract
where the NFTs for this particular painter are being stored.
Not that someone else can do that.
And that's what a lot of people did.
I mean, a lot of people tried to cheat you by selling you
one of these apes, but it was just a copy.
You always have to check, is this really from the smart contract,
from Yuga Labs, from the originators?

I'm not sure we can check, I think there's also being listed
here in the smart contract...
No, it's not a white one...
And...
...this white one.
And...
Yes.
I'm sure I see a look here,
a completely white one.
As it says something there, it's being...
...contract creators of those cars,
and this go...
Yeah, let's see if we click on that.
There may be that there's also been written
that it's somehow in clear text,
that this is...
...somewhere...
...not there.
No, it's not there, so you can't prove it there.
**Sometimes you see that there is some clear text, to identify the people who actually deployed the smart contract for that.**

### NFT Generator

So is this all magic? No, it's not.
What you can easily do is the following.
I'll just put it back here so that I can see something.
So, this is for example now...
So, this is now **a simple program that allows you to create NFTs**, to be programmed.
So, Ivan said that you can like normally you can either have your smart contracts
and what you normally have then is a frontend for your smart contracts,
somehow a webpage or application that provides that, that provides access to that.
So, this allows you now... with the simple **NFT generator**,
so you can create any kind of thing here, let's say a certificate.
You can give it a name, so it's lecture...
...NFT...
...example...
...lecture...
...who is the recipient, so it's me.
We could also, if someone of you has an address,
we could also type in the address.
We can select here from an image, let's say...
...take the test build, yeah.
And then we can add some traits (= metadata).
Let's say we do the first trait is lecture.
And today is the...
...21st...
...20...
...ok, another trait is topic...
...NFT.
Okay, and now we can also see here the JSON, yeah.
So what we do now is that we create these kind of JSON here.
And now we **mint the NFT**.
Oh yeah, so it's not there.
So it's being minted, what means that we now...
...address this particular smart contract that actually has been written by Ivan, and I'm just using it.
And this particular smart contract is now being addressed.
I'm sending this kind of JSON to this smart contract and it tells me...
...okay, the transaction hash, we can check.
It's the token number 982, which means that we created already 982 NFTs with this particular smart contract for a lot of different purposes.
And now we can see the transaction on [Polygon Scan](https://polygonscan.com/).
And that's here the transaction, yeah.
So it's now in block, we put it on Polygon, not on Ethereum, but on the second layer chain.
And there we see the information.
So it **costs us 0.02 Polygon**.
So all together 0.03 dollars to do this.

### OpenSea

And we can also check it on **OpenSea**.
So, here it is.
So, that's it.
So OpenSea is a platform where you can exchange and sell NFTs.
And I didn't register at this particular platform.
I didn't do anything with this particular platform.
I just created an NFT according to the appropriate standard, ERC721.
And since OpenSea is scanning **Polygon** chain for all smart contracts that belong to this particular standard, they somehow scan the whole blockchain and they show everything automatically that belongs to this particular smart contract.
And now it's there.
And there you can see that it's BCRL, so Blockchain Reallabor, that's our address.
NFT, this is something that Ivan mentioned in the smart contract when he created it.
He said that belongs to this particular collection.
And we see here the different Merkmale, let's see, that's something else.
But we also see the...
Can I see the times?
Normally I should also see the different traits (= metadata) that I created for this particular NFT.
Here it is, yeah, obviously. Created by, Lecture, 21st on January, NFT type, certificate and topic NFT.
And this is also now, if you would like to create NFTs for example, for your club for everything, you can create all these NFTs automatically.
And then they are automatically listed here.

And in the next step, what you can do then, is that you can also, if I would connect my wallet to this one, **I can transfer it**.
What happens now is that some people send you offers for that.
So it happens from time to time that people send you an offer to **buy this particular NFT**.
Very often it's fake and you have to be careful because it's fake, because nobody should be interested in this particular thing.
But it means actually, this is not an NFT that we created here for any value, it's more NFT that we create for a particular kind of purpose.

### Token-gated access

So what I can do now is that if I distribute this NFT to all of you, next time when you would like to enter the room, I ask you, show me that you own this NFT.
Or we have some kind of **token-gated access**, so that means that you would like to access a web page where I put the answers of the exam questions.
And you can only access that if you own this particular NFT.
Which would mean that then the web page asks you, please open your wallet and then the wallet needs to provide its address and then you check if your address appears here in the NFT.
And that is then so-called token-gated access.
So it's no longer an **access that depends on user ID and password**, it's an **access that depends on the ownership** of such a token.
And that is where all these different things now become really interesting also in business applications and in other kind of applications.
Because this provides you very nicely individual access to something.
Because you don't need to have user ID and password that you give away, if you want to share this with someone else, you only can do it by transferring the NFT.
And that's the way to do that.
Yeah, that's how you can do this in a very easy way.

### Streamlit: NFT JSON builder

And this is all done here with this **"NFT JSON builder"**.
And all in all, this is, I'm not sure,
did you come across **streamlit** already in other kind of applications?
If you want to do Python, I am now a fan in the meantime.
So if I do some programming, I do it in Python.
But then I do it in streamlit.
And streamlit allows you in a very nice way to realize very easily and very nicely any kind of Python application together with a very nice user interface.
So it's very often used in data science and AI applications for accessing large language models and so on.

And you see here are just the different Python statements to access here some kind of information.
So for example, this is here now to get the contract details,...
of addressing here the Polyscan web page.
And for example, if you just would like to get now the total supply, you address this by addressing the `contract.functions.total_supply()` and `total_supply()` is actually a method that is being offered by this particular smart contract.
So this is somehow the interface here.
You have smart contracts that have different kind of methods that you can also see on the web page.
And then you have, here in Python, public libraries where you can access the different functions by their name.

student3: What library are you using?

prof: This is the `web3` library.
Let's see what I'm importing here.
`web3`, yeah.

student3: So using that library you can interact directly with your smart contract?

prof: Yeah. You do that, you do that and you have to provide the,... you have to provide the **ABI**, which is somehow describing the interface to the smart contract.
So it's a meta description of the smart contract.
ABI stands for...?
Ivan, you know that?

Ivan: No idea.
So, last time I showed this JSON file which you see when you deploy the contract in Remix, there is ABI.
Basically, you see this **JSON representation of the smart contract**.
I mean, JSON representation, it basically means the function names inside the file.
So you know, when you call this function you need to call the smart contract exactly at that place where the function is.
When you click on the smart contract...

prof: Yeah, that's the...

Ivan: No, this is not the function.
We have to do the...
Yeah, exactly this...
Interactive tool...
Yeah, exactly...

prof: Okay, so that's the contract.
Yeah, that's the smart contract that Ivan deployed.
And here you can also see the different things that I can do with the contract.
Yeah, and here you see the ABI.

Ivan: So the contract...

Prof: That's the contract ABI.
So this is what I just took, I copied it into my Python environment and from that on, Python knows how the smart contract can be addressed/accessed.
So the blockchain itself has no API where you can say, well, what's the API of this particular smart contract?
But this is being stored here.
Yeah, and this now tells the Python program how to actually interact with the smart contract.
And so if you want to interact with the smart contract, go here to Polygon Scan or to any kind of other scanner, check if they publish the ABI.
Not everybody does it.
Some people don't do it.
They just want to hide it. And then you copy it into your Python program and then you can start programming again.

student3: Is the ABI derived from the bytecode?
Or is it the other way around?

Ivan: Yeah.

prof: Yeah.
Okay.
Good.

### slide: Is an NFT limited to art objects?

So, yeah, that's it about the different kind of NFTs.
See, yeah, can you go?...
Yeah.

Okay, yeah, you see, actually, the whole thing is **not limited to any kind of art**.

So we can also put **documents** on this.
Imagine that you get your final **master degree as an NFT**.
So the NFT is somehow being stored somewhere and you get the NFT handed over and then you can show this somewhere else.

You can have **machines** (as NFTs).
Everything can **become an NFT**.
And then, what we did in our lab is that we had a Master's thesis on that, Bachelor or Master's thesis, to have an inventory where every object that we have, like a **laptop or projector**, **becomes an NFT**.
And then, if you want to borrow something, you just borrow the NFT.
So and then you can check who currently has borrowed what. You just look into the wallet and then you see, ah, this is what Ivan currently has been borrowing.
And if he returns it, he just makes a return transaction, a transaction back into a wallet that represents the storage or whatever.

A lot of people do this with **commodities** like for example, resources like CO2, oil or gold or whatever.
So for example, one of the biggest examples is **Everledger**.
Everledger is a diamond.
**Diamonds** are being represented as NFTs.
And then you may argue **how can I actually map a real world object like a diamond to an NFT?**
And then we often get into trouble.
Yeah, so for example, how would we map this particular laptop here to a NFT?
The only thing is that it has a **serial number**.
That's its **unique identifier**.
With a **human**, we could do **fingerprint or DNA**.
With a **diamond**, actually a diamond has some kind of unique fingerprint.
If you put it **under light**, it has a very certain kind of **pattern** that it produces.
And that is being used to track diamonds individually.
And the idea was to identify blood diamonds.
So blood diamonds that have not been digged in a normal mine, but under other circumstances.

But the big trouble here is always, if you have now, let's say you have a **piece of land** from where you produce chocolate, cacao, or bananas or whatever.
Like you say, this is being **produced without any child labour**.
Yeah, so this can be approved.
That means when this piece of chocolate comes from this particular piece of land,
then it's being approved as being not associated with child labour or whatever.
And the **problem** really is that **who guarantees that there are not two smart contracts in the world that both guarantee that for the same piece of land?**
That's also always this kind of problem that people have there.
It's really, on the one hand, blockchain is unique,
but **it doesn't hinder you to have two times the same truth on the blockchain**.
And both refer to the same truth.
So it would be something like one global smart contract,
unique representation of each farmland and from there on you have the **traceability**.
So that makes it,... at the beginning a lot of people oversold the whole idea
and there was a lot of mistakes also with this NFTs.
So people said, well, we can now make all this whole chain, the supply chain trustful.
On the other hand, **people cheated** with **double entries** and things like that.
So we always have to keep that in mind.

---

Here’s the most natural and contextually correct way to fill in the gaps:

---

And both refer to the same truth.
So it would be something like **one global**, smart contract,
unique representation of each farmland and from there on you have the **traceability**.

---

Why these fit:

✅ “one global (smart contract)”

* The problem described is **duplication of truth**
* The ideal solution would be:

  * a **single, universally accepted registry**
* So “one global smart contract” captures that idea perfectly

✅ “traceability”

* Standard term in **supply chains**
* Refers to:

  * tracking origin → production → distribution
* Exactly what they want for:

  * cacao, bananas, etc.

---

🧠 Key insight (very important limitation)

Blockchain guarantees:

* ✅ Data **cannot be changed**
* ❌ Data is **not guaranteed to be true**

---

⚠️ The core problem explained

Even with blockchain:

* You can create:

  * Contract A → “This land has no child labour”
  * Contract B → “Same land, same claim”

👉 Both exist
👉 Both look “valid”

But:

* No built-in mechanism to enforce **real-world uniqueness**

---

🔑 This is called the “Oracle / Identity Problem”

* Blockchain cannot verify:

  * physical land
  * real-world facts
* It depends on:

  * trusted sources
  * governance
  * standards

---

💡 Real-world takeaway

To make this work, you need:

* Standardized registries
* Trusted authorities or decentralized validation
* Possibly systems like Chainlink for data input

---

If you want, I can explain **how real projects try to solve this (e.g. tokenized real estate, supply chain tracking)**—this is where theory meets messy reality.

---

## org

Okay, good.
So this brings me to the end of the NFTs and we got the link to the IPFS that we discuss next time.
And for those who came late, next Wednesday I need to get to a proposal meeting for a project proposal that we submitted
and this date is not negotiable.
They just sent you a date and you have to come if you want to have the money or not.
So we agreed upon that the next lecture is online on Friday morning, 9 o'clock.
Next week.
Next week, Friday morning, 9 o'clock.
I'll send you an email and then we'll be online.
Okay, good.
And then we decide next week we also have some trouble,
but maybe we can do it then next week, 9 o'clock to have a summary lecture on things like that.
Okay, good.
So next two weeks, the next two lectures.
That's the last one, but there we have an internal meeting in front of the institute that I have to participate in.
If something changes, I will announce, but at least the next one is definitely online because I have to be in Munich.
Friday at 9 o'clock.
9 o'clock.
In one week.
Not the day after tomorrow, but next week Friday.
The...
Friday night.
So it's Friday.
But I sent around an email.
Yeah.
Will it be recorded or not because there are some problems?
You can record if you are.
That's the problem I can record that.
Yeah.
Nevertheless, you should show up.
Because you asked so many questions.
We still don't have the slides for the...
It says...
Remember, you are stated in the audience.
Okay, good.
Okay, so then thanks and see you next week.
Bye.
Thank you.
