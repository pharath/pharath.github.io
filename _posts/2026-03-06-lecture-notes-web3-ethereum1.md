---
title: "Web3 and Distributed Ledger Technology - Ethereum and Smart Contracts (Part 1)"
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

## Bitcoin Lightning Network

First we want to have a look at **lightning**.

Yeah, the exam cannot yet be registered.
Is it this?
Yeah, that's right.
I will tell you, it shouldn't be a problem, hopefully.

Then, no, what we're going to do today is that today we'll discuss the lighting.
There was a few submissions of which one was almost very comprehensive.
So we discussed this and discussed some questions around this.
Then I hand over to Ivan and then he will explain something about the theorem and smart contracts.
Then I sit back and then I prepare the exam.
That's the idea. Hopefully afterwards.
Then you can direct check.
You can check the register and then we get this done.
Okay.
Good.

So, yeah, just the reason **why I got across lightning** was that there was just before Christmas there was a meetup in Cologne.
During this meetup there was somebody providing me a bottle of beer.
That's the **"beer of Satoshi"**.
Yeah, sorry.
With this beer of Satoshi, you open it and then was in the top.
There was a **QR code** that you can scan and then you get some Satoshi's.
That raised the overall interest because it was immediate.
No transaction fees, nothing.
Then I got into that.
So I just played around with it.
Then also, well, it might be a nice extension also that we discuss lightning in general because it's somehow an add-on network to the Bitcoin blockchain and provides very fast transactions without any costs.
But it's special in some case and that's what we're going to discuss right at the beginning.

And also in addition, what I started then is as a hobby horse more or less to **run a Bitcoin node**.
So we discussed this and discussed the whole issue of well, running the Bitcoin node is so power intensive and things like that.
Nevertheless, is someone running a Bitcoin node? By accident? Ok.

So what you can do is the following. There are two different things that you can start with.
I remember that some time ago we bought two Raspberry Blitz.
Yeah, so I think one of you actually looked into that.
Raspberry Blitz is actually a Raspberry.
Does someone own a Raspberry just for playing around with it? Yeah.
So it's a Raspberry Pi, 50 euros, nice Linux machine that you can run for the IoT and almost as a server.
And in the meantime they are quite powerful, so they're really quite nice machines to play around with and to run these as a Linux server.
There are also some kind of additional operating systems that you can put on top of it, for example, for home automation and all the rest.
So as a computer science student, I can only recommend to buy a Raspberry Pi and play around with it.

So, I have several of them at home and we bought these Raspberry Blitz some time ago, but we didn't really put them in operation, just bought them and then they were just lying in the cupboard of a colleague and so we digged them out and we started to bring them up.
And this Raspberry Blitz is actually a Raspberry Pi that's coming pre-configured, but you can do it with any Raspberry Pi.
And what you need is some kind of a large hard disk.
So you need an SSD, most preferably, of about a terabyte.
That's probably more expensive than the Raspberry itself.
And then you attach that to the Raspberry Pi and then you have a small Linux computer.
You also get a display on top of it, but you can use more TFT display.
So all in all, it's about 75 to 90 euros.
And then you run Raspberry Blitz on it.

And what Raspberry Blitz is doing, it's setting up a Bitcoin node, not a miner, setting up Bitcoin node, full Bitcoin node on the Raspberry Pi.
And this happens within about three to five minutes.
So how you do this, you actually download a image, you put it on a SD card with some special software, but that's easy to do.
And then you insert it into the Raspberry Pi, you boot it and then it comes up and then it says, hey, I'm running Raspberry Blitz.
And I have to set up myself. I'm doing some network connections.
And then it starts building the operating system.
Maybe reboots a second time and then it's up and running.
And then it's a Bitcoin node.
And what it does then is, then it starts retrieving the Bitcoin blockchain.
And it's not just retrieving it, it's retrieving it block by block and it's validating it.
For the sake of not just taking a Bitcoin blockchain, I mean all the data and taking it for granted, it's actually taking the first block, validating the first block. Then the second one, validating it, etc. - and validating means, looking at the transactions, building the Merkle trees, comparing the links, checking the nonce and all the rest.
So at the beginning it goes quite well.
So it starts up and running and then you see one block, second block and then wow, it goes up.
And then it **starts slowing down**.
Do you have an idea why it starts slowing down?
Giving the whole process that it's downloading block-by-block.
So after about a 100,000 blocks, after 150,000 blocks, it starts really slowing down.
Any idea why?

student: Maybe there's only limited amount of download capacity for the block and delay the block often.

prof: Yeah, it's what it's actually doing.
I cannot really connect it because it's running in my home network so I can't connect to it.
But it's actually showing that it's connected to several other nodes.
So it's really doing a broadcast on to some other bitcoin nodes and then it's connecting to the nodes using the Tor network (part of darknet) or clearnet (see "clearnet" vs "darknet") network.
But that's not really the case.
It differs.
So in some blocks you see currently connected to zero other Bitcoin nodes, then to 10 and to five.
But that's not really the effect.
Do you have any other idea why it's getting slower after some time.
It's not getting slower and slower.
It's just getting slower in some different periods.
When we look at Mempool, what do we remember from Mempool?
In Mempool we looked at the first block.
How many transactions did it have?
On the second block one transaction... yeah, you have an idea?

student: The number of transactions on the block is...

prof: It is increasing. That's the problem.
Right at the beginning, the first blocks, they almost didn't have no transactions.
Or just the first transactions and then was just mining block by block and there was nothing happening on the Bitcoin blockchain.
And now we have a lot of transactions.
We have thousands of transactions per block.
And this means the block gets larger and you need to do more computations, although it's basically just hashing around, but for a Raspberry Pi it takes some time.
At the moment, it's at - and I'm operating two of these Raspberry Pis at the moment - so I'm keeping track of it because it's very simple stuff, you only have this display where it's showing something, but what I just did is that I... they do have some kind of API, so you can ask it for some kind of information about how many blocks it currently has.
And this time I brought this.
Hi, there it is.
Okay, so what I'm just doing is I just installed a small script on it that is looking at the number of nodes that it already downloaded and I'm sending it to my Telegram channel.
So somehow, living on the dark side of the moon with Telegram.
But it's quite interesting because with this Telegram, you can easily write any kind of bots that send you information.
So Telegram is ideal for that, although it's somehow a strange communication media.
It's used by strange people sometimes.
So, and then you see it's currently running two different nodes.
These are my internal IP addresses 241 and 225.
So at the moment, they retrieved 480,000 blocks.
This one started a bit later.
So we are about 90%.
And I started it before Christmas.
So it's almost two weeks now and we are about 20%.
So it really takes quite some time to get up these nodes and running.
So it's really somehow difficult.
It's not difficult, but sometimes you got stuck, so we have to reboot it.
So Raspberry Blitz is probably not that really reliable.
So it's sending me a message every hour except during night and then you see we compare this now, that was today at 11 o'clock.
So we were 6.44, 55 and then we were 882.
So we got about 200 blocks in the meantime that the Raspberry Pi just added to this particular Blockchain.
Now, once it has retrieved all the different blocks, it will be up and running.
So that's what it's actually doing.
And once it's up and running, I have my own Bitcoin node running.
It's not a miner, so it's not participating in the mining.
It's just participating part of the network and then I have a node up and running.
Why should I do this?
You have some ideas?
You have questions?

student: Yeah, like a comment.
So I currently set up my own home server and I did not implement it yet, but my current plan is to do something similar.
Set up a Bitcoin miner and then have a lottery bot.
So I can send, I play the lottery and then one hash is mine and then it says if it's a win or not.
It's kind of like playing lottery, but without actually paying the fee.

prof: Is there some software that you installed or did you just implement it by yourself?

student: It's currently just a concept, but I would think it should be possible.

prof: So you set up just the mining nodes?

student: Just the mining node, but I do...

prof: you just need to be happy that you...

student: Yeah, just like maybe with a Telegram bot or something.
And if you start take a "Los" and then it sends you back with the win or loose.

prof: But then you're not participating in the mining.

student: Well, only with one hash.
It's one hash.
And not a mining pool, of course, because I don't want to share the win.

Ivan: Maybe if you open Google, I can show you something similar.
It's called **Bitaxe** (a Bitcoin miner, "Bitaxe 601 Gamma").
It's something similar just for a bit generated one hash.
Right now it's 1.2.

prof: So what shall we do for Bit?

Ivan: Bitaxe.

student: First one, the gamma.

prof: Okay.

Ivan: So basically something similar connected and generated random hashed and just work with something.

prof: Okay.
So basically...
Yeah.
...generated.

student: There are people who **mine blocks with that** every year.

prof: Okay, so it's basically just betting on your good luck.
From time to time you will find the nonce.
Yeah.
So you get it (Bitaxe) for 179 euros.
Okay.

student: I have a question.
I want to ask why would I want to do a node?

prof: Okay.
Yeah, maybe I'm just taking the plot from you.
Yeah, okay.
Why should I run this node without mining? You have an idea?

student: Yeah, I mean you can validate the chain.
So it's more of a supporting network and it's controlling the other things and not necessarily dimension.

prof: I can't show you.
Yeah, but that would mean that I'm just participating.
That I'm somehow a good guy.
I don't have any benefits there.
I'm just okay.
I'm a good guy.

student: I mean you can kind of come through and contact us with your views.
Yeah.
You can send something back.

prof: Yeah.
Yeah.
It's again in this area of I just participate.

student: But the first part is obviously you're helping the network by relaying transactions, verifying blocks.
But the second part, at least for yourself, you help yourself by being completely self-sovereign and independent.
You can verify your own balance.
You can scan the entire blockchain without anybody knowing what you're doing.
So you don't need any third party software.

prof: Yeah, that goes in the right direction.
So the first thing is really you become part of the community to support it.
The second thing is that once you have your little bitcoin miner, your Rasperry Pi, up and running,
what you can do is you can connect to it and you can use that to initiate your transaction.
Yeah.
So you no longer need to connect to a other node to do that.
So you don't need to somehow trust someone else you connect to in order to submit your Bitcoin transaction.
Whether it's Coinbase, or any other of the big providers, you are your own node operator and then you connect your wallet to your node,
you're Raspberry Pi and then you start your bitcoin transactions from there.
So, for example, if you're a bank, it's highly recommended to do it like that because then you operate yourself.
So if you are doing just for your private things, what you can do is you can set up a **lightning node** on top of it.
And this lightning node then allows you just to have lightning transactions directly from your local computer. But that's the point.

And then there is also **umbrelOS**.
So I also started to install umbrelOS because it was then - by looking around you find it easily that it is also somehow recommended - and I run this on an old laptop, a 10 year old laptop, and it is now at about 25%.
So in the meantime, it overcame the Raspberry Pis.
And umbrelOS itself is...
So umbrelOS is a system that you install also on your computer and you don't run it in any Docker or so.
Yeah, I mean, it's not something that you run as an application on your computer.
You have to dedicate your computer fully to umbrelOS.
So it's an operating system.
And again, you download it to a memory stick, you put it into your old laptop or whatever, and then it boots from that and then it completely deletes all the things on the computer and then it uploads umbrelOS and then you are there.
And then you have something like a local storage.
That's the first thing. So what they do is they just put the system up and then it's running in your local home system.
And then you can upload images, music, whatever.
So it's a local file storage.
That's one idea. They say, hey, you are somehow independent of the cloud.
It's your own cloud server that you can use.
That's the idea.
So you can dedicate a computer.
You can dedicate your old computer to be a fully local cloud storage for yourself.
And what you then can do is in addition...
So you see what you can do to run on it.
And that's the most interesting thing is there is, you can install applications on top of it, like photo libraries and the rest.
And then you can run your own Bitcoin node.
So what you do is you install this particular app in umbrelOS.
And once you install it, it says, Ah, okay, now I'm starting to download the bitcoin blockchain.
And then it starts downloading the bitcoin blockchain, since it's a more dedicated hardware, normally it's laptop or something like that, it runs a little bit faster. So in the meantime it overcame,... it started about two days later than the Raspberry Pi, and in the meantime it overrun them.
The point is this old laptop, it only has 500 gigabyte hard disk.
Bitcoin at the moment is 800, possibly 800 gigabyte.
So any idea how it or what it does actually?

student: pruned blockchain?
So it only stores the most recent X number of gigabytes.

prof: So it's pruning the blockchain.
So it's removing blocks that are probably not so important or not necessary.
So just removing it.
And that's quite interesting because you can set it to a limit.
You can tell it, okay, use only 200 gigabytes for the blockchain on your hard disk.
And then it reduces the blockchain to 200 gigabytes.
So you can even run it then on the lower machine, but you don't have the whole blockchain then on your system.
And it may be that if you have a transaction that relies on a particular transaction that was in this particular block that you have no longer available,
I'm not sure what it's doing, probably it is reloading it or accessing it.

student: And how is it, they decide, you know, how pruning should be done?

prof: I hoped that you didn't ask this question because I can't answer it.
Do you know how it's pruning?

student2: It just removes the initial X number of gigabytes from the blockchain and keeps the full blockchain history back up to a certain point in history.

prof: Okay.
Okay. That's easy.
It was more easy than I thought.
Normally these kind of answers...
They somehow sound reliable, no, it's, yeah, that's how it's doing it.
Thank you.
Yeah.

Then the question is what happens if I use a Bitcoin that has been transferred in the first number of blocks, it's probably downloading it.
I'm not sure what's going on.
I was probably saying, well, can't use that.
No idea.
Something to check.
Okay.

So long story short, that was the idea why I started with this lightning stuff.
Yeah.
And it's quite interesting.
So it's something like a hobby horse that you can run.
And unfortunately it really takes some time.
I thought that I probably can demonstrate you already during the lecture that it is up and running and then I can connect with my wallet to my own Bitcoin node.
But that's not yet the case.
Maybe in the lecture in two weeks time we are there. Let's see.

But then the point was why I actually started this was the **lightning network**.
Because what you can do then is that when we have this kind of lightning node here we can start **transmitting Satoshis using the lightning network**. And that's quite cool then in the end.
And I thought, okay, maybe we can use this also as a demonstration, which is not yet possible, but maybe later on.
And then I thought, okay, let's, let's dig into that one.

And yeah, there was actually one presentation that I thought was quite interesting.
And that gives us quite a **good introduction** to that.
That's by Mark.
Yeah.
Okay.
Perfect.
Would you like to give us the presentation because I like it and then you can tell us through.
Okay.
Okay.
So maybe you can give us presentation.
Yeah.

student1: So I'm presenting you the **introduction to the lightning network** and why we need it.
So the Lightning Network tries to solve some trade-offs that the different option has,
as it is sometimes really slow, and needs approximately 10 minutes for the confirmation,
and no instant payments are available, and even if the network utilization is high,
you have to pay high fees. And also the smallest tuner is Satoshi, and once Satoshi is
approximately 19 cents currently, so you can't make small payments. And that's why
the **Lightning Network** has been introduced. So the Lightning Network has a second layer
and doesn't change the Bitcoin blockchain base layer. You only have to make a transaction on the
Bitcoin blockchain for opening and closing the channels, and the Lightning Network consists
of channels between two parties, and we are now looking at one payment channel and how these work.
So at first **to open a payment channel**, you have to make a transaction on the blockchain.
Here for example, Alice and Tom are sending each two Bitcoins into a multi-signature wallet,
and this multi-signature wallet is off-chain in the Lightning Network, and inside the payment
channel in the Lightning Network, they can make unlimited transactions and transfer also
mini-satoshis, and each of them has a channel backup of the bitcoins that has been transferred.
So here at first, both have two bitcoins, and then Tom is sending one bitcoin point to Alice,
they both update their channel, and if they agree on **closing the channel**, there's another transaction on the blockchain,
and there's then the status that Alice has three bitcoins and Tom has one bitcoin.
So, yeah.

student2: (question)

student1: I don't know how, that's,...

prof: there must be an idea, right, so again,...

student2: so in the main one, the connection is going to be good,
so we can go down with two of them, and here, the connection is going to be good.

prof: No, I think, at least what I thought, I tried it, and it was just integers,
but maybe they use internally also...

student3: You can transfer milli-cents.

prof: You can transfer milli-cents?

student2: And then the question is, how do you control the channel?

student3: I think it gets rounded... to one Satoshi.

student2: So it's probably up to one Satoshi.

prof: I mean, we are talking about cents.
Some losses are there, I think.

student1: Okay.
So, yes, in theory, the Bitcoin is the base layer. It's used as a secure settlement system.
It's decentralized, but has limited throughput,
and that's why we have a layer two lightning network, it's built on top of Bitcoin,
not changing base layer, and it allows for instant low cost payments,
and the lightning network consists of plenty of trading channels between two parties,
and only to open and close the two payment channels, we have a transaction on the blockchain.

prof: So, there are some, thank you, no, but you can sit down, thank you, yeah.

student3: May I add one thing?
I just wanted to add that one of the other main motivations behind the lightning network
is the limitation of Bitcoin's block size, because it could never be a global payment system,
with Visa, for example, doing tens of thousands of transactions per second,
Bitcoin would never be able to reach that, so it needs the lightning network because of this.
Yeah.

prof: So, in some of the presentations, most of you came up with San Salvador,
and in San Salvador they use Satoshis and the bitcoin lightning network.
You can buy a coffee there, and at least a colleague of mine has been there on holidays,
and he said, yeah, I could buy my coffee using bitcoin there, they do that.

The point is now, when we look back here at these different things,
and when I came across that, I think that's from the, and therefore, I selected this presentation,
it explains very nicely what you do, actually you both, first, you need somehow,
you need to upfront, you need to upload something into the lightning network.
Is the lightning network a blockchain? No, no, it's not a blockchain.
Could be a nice **exam question**, no?
So, is it a blockchain? No, it's not a blockchain, it's just a network on top of it.
And within this network on top of it, what you do is that two people just open up a channel,
and they need to upload data or some kind of money into the channel.
In this case, it's two people, they upload two bitcoins each.
How many can they transfer then within this channel?
Maximum four, nothing more.
They cannot transfer 10 bitcoins, because it's just they are somehow operating more or less on a bank account
that they just opened, so you can compare this somehow with a bank account.
And from then on, we basically just use a kind of database operations, nothing else.
And in the end, we do settlement, and then we put it back.
So, the point is now that I got,... and let me go back here to see...
Okay, so, later on this morning, so I'm sure you've ever seen correctly...
Is someone else having a lightning wallet? Yeah, but okay.
Good, so that's the wallet of Satoshi.
Yeah, so it's the wallet of Satoshi, it's one of the easiest one.
That's what I got with the Satoshi beer.
And it says that in my wallet of Satoshi, I have about 150 Sats, Satoshis, which is about 14 cents.
So, about two hours ago, I just transferred 150 into it,
because that's now, and that's new somehow, or not that new,
but that's now a self-custodial wallet that's running really together
with my passphrase on my own phone.
I also have a custodial wallet, which is run by the wallet of Satoshi network,
or the wallet of Satoshi providers, where you see I have 160 Sats.
All together, with this beer of Satoshi, I got 310.
That's what I got with the bottle of beer.
So, I paid to the other one.
So, what I can do now is that I can start sending around,
and we can try how to do it, send something,
and now if you have something, we have, shall we do it, shall we try it?

student3: Yeah, sure.

prof: Yeah, so, then you see how it happens.
What I need to know now is just I need his...

student3: Will you send me or I send you?

prof: I send you, last time you sent me, now I send you.
So, if you like, you can just download the wallet of Satoshi app.

student3: I'll make a lightning invoice for 100 Sats.
Yeah, or 50.

prof: Oh, no, do something with milli. Do something, can you?

student3: Sure, even smaller.

prof: We can try this and then we can...
Okay, that's really...
So, what he's doing now, he needs to develop,
or he needs to **create an invoice**,
which means that he wants to get money from me,
and with this invoice, he transfers, or in this invoice,
he **encodes** his **wallet address** and **the amount**.
So, I scan that.
Okay.
And now it's asking me, I can put a note to it, lecture.
Okay.
And then I can send.
Network problem.
Uh-oh, uh-oh.
We'll try it. Then we do it again.
Yeah, I'm online.
I have a set on the bottom of the page.
We cannot have this amount of Satoshi.
It's too small. 0.165.
Should we try a bigger amount?
Yeah, **try a bigger amount** with some...
With some...
Yeah, that's what I thought.
So, 50.
Yes.
Okay.
50 Sats.
Okay, send.
Okay.
You got it.
Instantly received.
Instantly received.
Yeah.
And that's quite a nice way.

student3: Do you want it back?

prof: No.
It's the last time you've sent me something.
Okay, so...
I even got it for free.
It was a deal.
So, that's an easy way to do it.

But now the question is,
I never opened up the channel.
At the beginning with this tutorial,
the idea was, you both have to open up the channel.
But I never opened up the channel.
The only thing I did was,
I opened up the bottle.
There was a QR code.
Then I got an introduction on the bottle label.
That you have to download the Beer of Satoshi app,
which I did, scan it, and then I got it.
I opened up the wallet,
but I never opened up the channel.
How did this work then?
Is this false what he told us?
Or what happened?
Any ideas?

student: So, you can have this,
you can try and you do not have to have both parties.
And that's something initially,
so it could be the near part of your channel
that actually couldn't be somebody's.

prof: That's also what I thought.
That I just opened up an empty channel.
But I never created somehow a Bitcoin address there. It just happened like that.
I think, yeah.

student3: So it uses existing channels.

prof: Yeah. It's just using the wallet of Satoshi channel.
So basically, what it's doing,
and then we come, again, back to this whole custody level.
So actually, the first wallet is a custodial wallet.
And that was very interesting.
When I downloaded the wallet of Satoshi app,
it only asked me for a password,
but it never showed me a passphrase.
Which means, hey, I never created a real wallet.
What I did was just somehow a log-in
to the wallet of Satoshi server,
which is managing more or less just a database.
So what happened is that I just created a wallet of Satoshi wallet,
which created a log-in to the wallet of Satoshi server where now 310 Satoshis were marked as mine, out of the thousands of Satoshis that are owned by the Satoshi beer people, or by the wallet of Satoshi people.
So actually, it's one big lightning channel
that is somehow being shared by a lot of people,
but upfront you have something like a client
that just manages somehow internally
how much you actually own within this particular channel.

student: So if you want to spend money, what lightning do you open?
If you want to buy groceries from now on with lightning, I would...

prof: Not necessarily.
What's happening now was that he...
I think you own a channel?

student3: Yeah.

prof: So he owns a channel.
And what's happening was that not,... the transaction was not done
between my channel and his channel.
It was between the wallet of Satoshi channel and his channel.

student3: It was routed through other channels.

prof: Maybe also routed through other channels.
That's the second level of content.

student: Okay.
So receiving the wallet of the channel,
if you're spending money...

prof: Receiving goes without a channel
if you use an app like wallet of Satoshi.
Where you just open up a custodial account, more or less.
You can do something like a breeze
and there are some other kind of wallets.
And they require that you actually connect yourself
to your own lightning node.
And that is something I wanted to show you,
which is not yet possible because we don't have yet one.
Maybe in three weeks time we have one.
But that was the idea.

And if you open up now,
if someone of you downloads the wallet of Satoshi,
I can send you something
and we just pretend that we are sending something.
Actually what you do is you just put a database entry at the wallet of Satoshi server.
So we're not going to send.
But okay, you have questions first.

student4: So this is basically very PayPal?

prof: It's PayPal.
It's PayPal with Satoshis.

student3: I was just going to say the app that I use, Phoenix,
you can actually open a...
They open a channel for you, but it's your own channel.

prof: That's what you can do.
That was then the next point I was researching.
What happens if I don't have a lightning server?
I don't a lightning node.
And then again you need to connect to someone else.
And then you go to...
What was it?

student3: Phoenix.

prof: Yeah, Phoenix.
And there was Phoenix and Breeze.
You go to Phoenix and they provide you with an access to a lightning server.
And there you can open up then the channel.
Are you using it from time to time to do some payments?

student3: Yeah, I do actually, to buy some stuff online sometimes.

student4: What?

(laughter)

prof: That's my question.
It was a question.

student4: So let's say I have a word for Satoshi. How do I add this?

prof: Either by asking someone else to do it,
or you really open up then and not that far,
you really buy Bitcoin more or less from a Bitcoin provider or something like a Coinbase or whatever way you buy Bitcoin.
And then you upload them into your channel.
Or you somehow insert them into your channel.

student4: So if I have like some bitcoin on my actual wallet I can transfer it...

prof: Yes, you can transfer it.
When you have some Bitcoin, you can transfer it into your lightning wallet.
And from then on they are somehow locked there.
And when you get more or less,
then at the end,... actually what as far as I understood it, maybe you know better...
But what you do is, let's assume you have a Bitcoin.
And then you move it into your lightning wallet.
Which means that then the Bitcoin is somehow really being removed from the Bitcoin blockchain.
You can't use it anymore.
It moved to your lightning wallet.
Then you can make transactions.
And when you completely...
Let's say you transfer half a Bitcoin to me and half a Bitcoin to him.
And in the end, yours is gone and we then settle it back to the Bitcoin blockchain.
And in the end, we have half a Bitcoin each on the Bitcoin blockchain.
But we do it completely outside.

student4: So when I send money to... I create a channel.

prof: Yes.
I mean, you don't really, to a wallet of Satoshi, you don't create a channel.
You actually send it to the channel of the wallet of Satoshi guys.

student4: So I create a channel with the...

prof: It's actually...
You use **their** channel.
What you actually do is...

student4: Because we just...
If I want to add money to the channel, I don't have to create...
Which means I have to create...

prof: Okay.
When you're creating a channel,... when you're talking about a channel, you always have to consider, is it my own channel?
I created it like he did.
He created his own channel with Phoenix.
Using Phoenix, let's say.
So we have the Bitcoin blockchain.
And then we have at least one of them is a miner or maybe just a node.
And he's providing a lightning node.
So with this lightning node, now some other people can create channels.
So each has a channel.
That's what he did.
He created a channel with this lightning node provided by, let's say, a few people.
So, now, and we also have, for example, someone who is running...
That's me with my Raspberry Pi.
And I have also a lightning node and I also have a channel there.
But just me.
I'm not offering that to anybody else.
And then we have all the guys that just accessed because there's also a node, the wallet of Satoshi environment.
So they also have a channel.
And they have a lot of people there.
But they are just accessing this channel, but there is a database
and that database is taking the account.
And that says, well, he has one S, he has 10 S and he has 20 S.
But this is just listed here in this particular account.
So it's the first thing.
And they didn't really open up, with the first instance of the wallet of Satoshi, as I said, I didn't even create a passphrase.
I just had to log in into this database.
Then this morning when I started, then I said,
okay, now I would like to make a transaction.
I want to send something.
The wallet of Satoshi said, hold on, you can't do this any longer.
It's no longer possible in your region.
I don't know why, maybe some regulatory issues or so.
I said, now you have to create your own wallet of Satoshi, custodial wallet of Satoshi.
So now actually I have two wallets on my iPhone.
One is the custodial wallet and one is the non-custodial wallet.
This one, the custodial wallet is basically just the database entry.
The non-custodial wallet, as far as I understood it, is really something where I get direct access to this channel.
But more or less, I'm just using their channel to move things around.
We are not moving,... but when I'm doing the transaction between my two wallets,
I'm basically just moving things around here in the database.
My transaction to him really moved something from the wallet of Satoshi channel
to the Phoenix access point with his particular channel, because he has his own channel.
So that was really a channel to channel transaction.
All the gimmicks that I did was just putting database entries around.
And not yet any Bitcoin transaction took place.

student5: So also maybe why Bitcoin is really essential,
because you said "you need a channel for a transaction, but you need a transaction for a channel"
so to create a channel you do a Bitcoin transaction on the blockchain
and then do something in the Lightning network,
you can then use this channel, it was created from the transaction on the Bitcoin blockchain,
so you don't need Lightning to make Lightning.

student6: No, but like you have your channel, but like you want to have more money.
So you have to close this channel and you create the new one.

prof: No, you can just upload Bitcoins to this channel.

student6: Ah, okay.

prof: You can ~~spessed the~~ put it up.

student6: Okay. And how does it work with the wallet of Satoshi?

prof: Uhm, I assume,... I'm not sure if you can do it with the very simple wallet of Satoshi,
you can fill it up with Bitcoin, I've tried with that, I don't know, I have to check this.
I think with the custodial wallet is that you actually fill up Bitcoins
into this particular channel to which you have access with your wallet.
It's somehow that they somehow manage your particular bank account
within this particular channel.
But I'm not sure maybe I'm not sure maybe with my wallet,
with my custodial wallet, with my non-custodial wallet,
so my own one, maybe with my own one I can even have a channel, but I don't know.
Something to test.

student7: So how I understood from my research, is that the channel always needs two parties.
And that you need a multi-signature transaction on the active blockchain
to create a channel between two parties.
How would you want to manage to upload actual bitcoin on two existing channels?

prof: Good question.

student8: So I think there's probably an address and transaction on the main chain
and if you send more money to that address,
you can in the Lightning network verify that there was more send to that address
and then you probably spend more money on the Lightning network.

### Time Locks

student7: But you have multi-signature release,
a contract that when you close off the channel,
you are **time locked**.
So then the party that closed off the channel is time locked
in order to prevent the party to publish more beneficial state or an older state.
For example, let's say we open a channel to `true`,
I give you one bitcoin, then you have free one,
so I want to publish the older channel to do the same thing.
Then the release is time locked.
But if you do an additional transaction to the same channel,
you would need to create a second, then you have two parallel release contracts.

student8: Yeah, maybe, I don't know.

prof: I don't know either.

### move between the different channels, multi-channel transactions

But something that is interesting is the point,...
what happens if,... since we share the channel,
he's on a channel at Phoenix, I'm on a channel at wallet of Satoshi,
**how did we move between the different channels?**
And then you need to have a **multi-channel transaction**,
which goes with a complicated algorithm,
which nobody really of you addressed.
I also jumped into it. Does somebody really know how it works?
It has something to do with, nobody can...
What's actually happening is that if I want to transfer some money to you,
and we can't do it directly, we go through his channel,
but it could be that on the way he steals some of the Satoshis,
that must be ensured that that is not the case.
And there are some algorithms to do that.

student: (question) time locks?

prof: Yeah, there are some time locks,
but they also use some kind of **zero-knowledge proof**,
so the final recipient has to be provided with a **secret**.

student: Yeah, the secret.

prof: Yeah, right.
So the one who is actually receiving the money needs to know a secret, which is then being provided to all the rest,
and only when you know the secret, the money is being transferred to you.

student3: The pre-image of a hash, yeah.
That's the secret.

prof: That's probably also something, maybe, another exercise on.
It's not part of the exam.

### Summary

But something that you... I think what is interesting to remember,
and that's why I mentioned this whole... **why I mentioned this whole issue** really is that,
**yes, there is a solution really to use Bitcoin as a kind of payment media.**
Yeah, it's something **like PayPal**.
It's **a PayPal on top of Bitcoin**, that's also something to remember.
It's **not a blockchain**.
The **Lightning network is not a blockchain**.
And **you can really operate it more or less by yourself**.
And something that is interesting, that I think maybe we can reach it within this lecture, otherwise we just do it ourselves,
is really **to run your own Lightning node**.
And with your own Lightning node then from then on, **you really can operate local transactions**.
So it may be something that you start working here on in the Aachen Blockchain Club.
So next time you have a Blockchain Club meeting,
you just do some kind of Satoshi transactions for the beer you buy,
but you just do it locally.
Completely locally.
You **don't need any stablecoins** and all the rest...

Okay, good.
So there were also some other kind of submissions talking about some basic issues
and also how it's being used.

### Statistics, Distribution over the World

Did you one where you said **the distribution of the world**, there were some statistics?
Somebody did, so the one with statistics,
and that most of the wallets actually exists in the United States.

student: Yeah, like the absolute majority of the Bitcoins in the network are in the United States,
but I think what is more accurate for looking how much this is adopted
is the amount of nodes.
And the absolute majority is in the United States, but not as much.

prof: Is there this one, though?

student: Yeah, that one (submission).
Yeah, there you can see the amount of Bitcoins in the network,
where I think about 60% of all Bitcoins in the Lightning Networks are in the United States,
and then it goes down.
But I think this is the more accurate statistic for how much it is adopted by common people kind of,
because, for example, in Germany, about 350 nodes operate,
and there we ..., that's a bit more in the distribution.

prof: Is that just recent data?

student: From,... according to the website, from yesterday.

prof: Ah, okay, good, yeah, yeah, you got it.

student3: If you go to mempool.space you can see live stats on the Lightning Network.

prof: So it works out, okay, we can do this, then.

student2: I'm wondering because the...

student: I speed read the presentation, so I only looked at one website.

student2: They had the extensions for Bitcoin Lightning.

student3: They banned it, I think.

### Story about Project with China

prof: Actually, yeah, in China it's not allowed to do any cryptocurrencies.
Except for,... actually, it's quite a nice story.
We did a project with a local German company,
and they sell particular kind of products in China,
which are then being manufactured in China and then delivered to the United States.
So what they deliver is a high-quality product, very expensive.
What happens?
The manufacturers in China, from time to time, they use more simple products.
So they wanted to prove that, at least the final customer in the United States,
they wanted to have the proof that the Chinese people actually used the German product.
And so we managed that with a blockchain application.
So what we did is this product that's being sold is some kind of foil.
The foil is in particular roles, and the roles are tokenized.
So we put a token on the blockchain and then the Chinese people use that.
They need to get a token out of that.
So, and then they deliver to the people in the United States, actually,
that this token is used for that particular product.
So it wouldn't make sense then,... so they have to prove that they got a token for this particular product.
If they just mix in some other stuff, they wouldn't be able to show the token.
That's the whole issue of that.
So we implemented it and they tested it with the people in China.
Then the people in China said, hey, hold on, we are not allowed to use that.
Because we are banned to use any kind of blockchain that involves sending cryptocurrencies away.
So what we argued is, no, you're not sending cryptocurrencies around, it's a token.
And I'm not sure how it ends.
So we wrote them a long statement that this is still allowed,
because they're actually not sending any cryptocurrencies.
What they do is just sending tokens around, which are not related to a cryptocurrency.
That was the first solution.
So we are arguing about this.
I'm not sure what the current situation is.
I'm talking to the guy next week, and then you see.
The other point is that what we could do is to install our smart contracts on the Chinese blockchain.
The Chinese are running their own blockchain.
And that would be allowed.
But that's probably not in our interest.
Providing,... moving the trust then just to the Chinese blockchain.
So that's quite an interesting issue in general, that we actually also haven't thought about it.
That we develop a kind of blockchain application, which in the country where it should be used,
and where it should protect someone, they just say, no, we're not allowed to use that in general.
So it's quite interesting.
The other point would be to do something like a custodial wallet, that could be the next solution,
to do a custodial wallet by saying, okay, the people in China,
let's say the people in this country, the people in this country are not allowed to do that.
They just,... they don't have a wallet themselves.
They just have access to a website.
And this website is then being hosted in Germany.
And this website is then doing somehow the transaction on behalf of them.
This means they are not actually doing any currency or cryptocurrency transactions in China.
What they do is they just use a website and they just even don't know that there are some cryptocurrencies behind it.
So we completely hide all the blockchain stuff. That could be the other solution.
But then there's no longer really a distributed app, which is at the moment.
At the moment, it's really a JavaScript app running in the browser that is completely communicating with the blockchain through a smart contract.
So there's nothing on a server. It's completely serverless.
Just a smart contract. There is the app that you have to download from a server in Germany, okay.
But then it's completely running in your browser, just communicating with the smart contract.
And they could have a point by saying that this is probably not allowed in their particular country.
We will see how this works.

### org

But that brings us now to the point that we are talking about next, distributed apps, or **decentralized apps**,
about **smart contracts**.
And with this, I hand over to Ivan, who is going to tell you something about smart contracts,
how to develop them, how they actually work on the **Ethereum blockchain**.
And he will do the rest of this lecture.
And then also the next one, because next week I can't come to Aachen,
because we have an important meeting in our institute, so I won't be able to come,
but he will take over then, and then next week I'll be back.
And we will also do then some kind of, I'm not sure if you can do it already today,
an exercise on **developing your own smart contract**, but we will see.

Ivan: Today we don't talk about this.

prof: Okay, today just the introduction. I took so much time again for other stuff.
And now I try to check the exam.

## Smart Contracts and Applications in Ethereum

ab 58:16

That's all about Bitcoin, I would say.
I never really understood why people spend minor amount of money for buying stuff with Bitcoin. Usually there are cheaper ways.
But in order to explore the cheaper ways you need to also explore other kinds of blochchains that are live. One of them is **Ethereum**.
So the idea of **today's lecture** was to give a little bit of recap
about how the blockchain worked, and then jump into Ethereum.
So before I show you how Ethereum works, and introduce you to the Ethereum ecosystem,
I will just recap basic stuff, because this is important.

### slide: Blockchain Recap - hash functions

I mean, I'm pretty sure you already covered in your previous lecture.
But you should already know that **hash functions** are one of the most important things,
they are used everywhere in the blockchain.
And should I explain it, or someone who want to just give a guess how they work?
Yeah?
Does anyone want? Or know how they work?
You should know.
No, I'm not going to talk about that.
Okay, I'll do it.
So we don't need to go into the details.
We just need to remember that we have an input and output, and that is the output is this **digest**.
And whatever input we give, we have the same length, the same size of output.
The only interesting thing is whatever small change we add into the input,
for example, we add a small letter or some sign or anything,
the output, this digest, will be completely different.
So in case we are sure that we can upload a whole book
and change only one letter of that book and you have a completely different digest.

### slide: Blockchain Recap - What are Merkle Trees?

Why is this important?
Because we need to somehow **store transactions**.
And how do we store the transactions?
We do it on the blockchain.
Pretty sure we already talked about this.
We store them in the **merkle trees**.
Because we cannot combine all the transactions into one book.
Basically that's why we get those merkle trees,
where they pack all the transactions into one small hash,
and we upload this hash on the blockchain.
Basically we upload a hash inside a block on a blockchain.
The **merkle trees** are different **types**.
So the simplest one is this **binary merkle tree**,
where we have **binary values**.
So two values produce one leaf.
Here you see another two values will produce another leaf.
The last leaf is basically called **root**.
This is root of the merkle tree.
And this is, as I said, the most basic one.
But **different bookchains use different kinds of merkle trees**.
For example, **Ethereum** uses a **Merkle Patricia Tries**.
**Solana** uses a **concurrent merkle tree**.
So they are different specifics of the merkle tree, they have different properties, but you don't need to understand that.
The basic stuff you need to understand is how they work,
and the **purpose** why they work.
So the purpose is you want **to combine information into a small package**.
Let's say you get a bunch of transactions,
and you pack it in a small **block**, a small root cache,
which is then stored in the block.

### slide: Blockchain Recap - What does a block contain?

And that brings us to the **blocks**, basically.
I assume you already covered how a block looks like.
But this is a simple explanation.
Not all blockchains have the same structure,
but let's say that basic, basic stuff is this one.
And this is how most of the architectures of the blockchains are designed.
On the bottom here, we have a bunch of transactions.
So basically those transactions, which we see, for example,
on the lightning network to buy some, I don't know, beer or anything from the blockchains,
or simply **sending money to one account** or to another account.
**By doing so**, we **create a transaction**.
This transaction, then here, basically on the bottom.
I mean, they are not in the block,
but those **transactions are basically in the payload/data section of the block**.
On Ethereum, we don't only send transactions from one account to another account.
On Ethereum, we have those so-called **smart contracts**,
which are really interesting.
We'll look into them later, probably next time.
But it's important to say now that whatever we do with a smart contract,
or whenever we interact with a smart contract, we create a transaction.
And those **transactions are all protected here inside the blocks**.

So now you know here are the transactions.
Those transactions, if we go back to the previous slide,
we'll pack them into the **merkle tree** and we get a root over here.
So basically here you see the merkle root of all the transactions,
which have been inside the block.
Then we have the **previous block hash**,
which is `#600` which is from the previous block.
We have the **timestamp**.
In Ethereum **you don't have timestamp**.
We use the Linux universal number for timing, it is basically a **integer number**.
And by this, you can determine what time in minutes and seconds it is right now.
And you have the **nonce**.
We already, you remember the last time, or some lectures ago we discussed the nonce.
Nonce is used for calculating the block hash.
And it's different based on the difficulty of...
This is for, let's say, proof of work.
Because proof of work is for Bitcoin,
proof of stake is for Ethereum.
So we can differentiate between those **consensus mechanisms**,
but this nonce is now used for in that case for the proof of work.
Any questions on this?
Any questions?
No questions?
No questions. Ok.

Yeah.
So you know how the blocks looks like.
Again, Ethereum is not used in proof of work.
Before it was using it, it started as a proof of work.
But then I think, in 2022 or 2023, they changed to proof of stake,
where we don't have any more miners.
Miners are strictly for proof of work consensus mechanisms.
In Ethereum, now we have **validators**.
And how and what's the difference?
Basically, miners are providing this computing power
to mine the new block, basically to create those new blocks.
On Ethereum, we have validators with stake money,
so basically **they deposit Ether (ETH) to create Ethereum**.
And when they create a new block,
they are rewarded for the creation of that new block.
But in that case, they are as I said...
Maybe I can go a little bit deeper later, how they work.
But for now, I don't think it's important to go deeper because you might get confused.

### slide: Overview

The overview is ~~ongoing~~...
Now I will come a little bit to the introduction.
I will explain you what is Ethereum,
what kind of cryptocurrency,
the currency behind Ethereum,
how a transaction looks like, how accounts are created,
and Solidity,... but I assume it's for the next lecture,
how the smart contracts are going to be built.
I mean, the smart contracts will be built.

### slide: Ethereum Overview

So, a little bit backstory about Ethereum:
It's the **second largest public blockchain by value (market cap) after Bitcoin**,

It is a **Turing-complete** blockchain,
meaning that you have enough amount of time,
and computing power, you can basically
develop every operation on it.

It runs on **nodes**, again, as Bitcoin.
But in that case, we have here something very interesting.
It's called **Ethereum Virtual Machine, or EVM**,
which is basically the operating system on those nodes.
You have an operating system on normal machines,
like Windows or MacOS or whatever, or Linux.
And here on Ethereum, it's Ethereum Virtual Machine,
which is the arc of the node.
It's basically doing all the operations
and doing all the magic on the blockchain.

As I said, it first used Proof of Work as a **consensus mechanism**,
which means how the blocks are created.
First started with Proof of Work, again, mining.
Lots of people were interested in mining Ethereum,
but now it moved to **Proof of Stake**,
where the increased energy consumption is reduced.
And yeah, it's more or less...

The main **cryptocurrency behind the Ethereum network**
is called **Ether or ETH**.
So for whatever use case we send transactions
or we interact with the smart contract
we use the ETH currency for paying the transaction.

And it's roughly **active**, I mean, officially **from 2015**.
Last year basically was a,... even here in Aachen,
there was a small party, so everything came here.

Yeah, so far so good, no question.
Okay.

### slide: ETH (ether) as Cryptocurrency

Yeah, so as I said, Ethereum has its own cryptocurrency
and it's called Ether.
And this is basically the economic incentives
for those validators to share their resources on the network.

How is this created?
Basically, whenever a validator creates a block,
he receives a **staking reward**.
This is how basically new coins or new currencies
are coming into circulation.

Those **stake rewards** are received by proposing blocks,
testing those blocks or participating in the consensus correctly.
By this, we create a computational market
where there is an incentivation for everyone
to participate and to support the chain.
This is more or less how almost all the chains are working.

What else do we have?
Whenever we send a transaction,
we have a sender and receiver.
And again, like in Bitcoin,
the sender needs to pay a **transaction fee**
in order for this transaction to be executed
and included in the block.
This transaction fee is, again, **dynamic**,
meaning that if we send simple transfers,
for example, money from one wallet to another,
we usually pay around 23,000 gas,
you will see this later on the next slide,
but if we **interact with** some kind of **smart contracts**,
we **need to calculate the transaction fee**.
So it's not a fixed amount,
**different transactions cost different amounts**.
Yeah, and again, by different transactions,
I mean different transactions with the smart contract.

So I think we discussed this when we sent money
from one wallet to another,
like two lectures ago,
but I will recap this briefly
because it's a little bit confusing.
The **transaction fee** is basically
**a combination of those three elements**.

We have a **base fee**, which is, as it sounds,
like a base, this is the fixed amount of money
we need to pay in order for our transactions
to be accepted on the blockchain.

And then we have a **priority fee**.
Priority fee is the amount of money
we are willing to give to the miner
to prioritize our transaction from all rest.
If you don't pay or we pay a low transaction fee,
probably our transaction will end up later on the blockchain
after two seconds or two minutes or in an hour
depending on how many transactions are there executed.

And then we have a **gas limit**,
which is the maximum amount of gas
you are willing to pay for your transaction.

So here is the thing.
Think about you are going to a petrol station
and you want to put petrol in your car.
The **base fee** says, okay,
on the petrol station, one liter of petrol costs,
let's say one euro,
but then there is a very huge queue on the petrol,
like lots of cars,
and then I say, okay, I will pay 150,
so that I go before everyone else.
So you have this **priority (fee)** to go faster.
But when you go to the machine,
when you start putting petrol in your car,
you put a **gas limit** saying, okay,
right now I am willing to pay for at most 50 liters of gas.
If it goes above 50, then just stop it because I don't get charged more than I want to pay.

---

#### ChatGPT: "does the gas limit include base fee and priority fee"

No — **the gas limit does NOT include the base fee or the priority fee**.

They are **separate concepts**:

##### 🔹 Gas limit

* The **maximum amount of gas units** your transaction is allowed to use
* Think: *“How much work can this transaction do at most?”*

##### 🔹 Base fee + Priority fee

* These are **prices per unit of gas**
* Think: *“How much am I paying for each unit of work?”*

##### 🧠 How they work together

What you actually pay is:

> **total cost = gas used × (base fee + priority fee)**

* **Gas limit** → caps the *quantity*
* **Fees** → determine the *price per unit*

##### ⛽ Back to the analogy

* **Gas limit (50 liters)** → maximum fuel you’ll take
* **Base fee (€1/liter)** → standard price
* **Tip (€0.50/liter)** → extra to skip the line

👉 Total cost = liters used × price per liter

##### ⚠️ Important nuance

* You don’t always use the full gas limit
* You only pay for **gas actually used**, not the maximum
* But if your transaction fails, you still pay for the gas consumed before it failed

##### ✅ Final takeaway

* Gas limit = **quantity cap**
* Base fee + priority fee = **price per unit**
* They multiply together, but they are **not included in each other**

---

So this is the example you should keep in mind
when you think about how the transaction fees work.

- two prices, one amount:
  - Fixed amount, priority amount, and this is **money (in €)** and **money (in €)** and
  - this (gas limit) is the **amount in liters** in our case.

student: And the **gas limit only applies to smart contracts**, right?

Ivan: Yeah, exactly.
So basically, we have...
Actually, no, maybe **also to small transactions**.
But the small transactions, I say,
if I pay something,
if I send from my wallet to your wallet,
I already know how much amount of gas I am going to consume.
So, this is more, I would say, for smart contracts,
for smart contracts, you just need to calculate
how much is the possible for this operation
for this activity to happen.
And then we can say, okay, there is some fixed smart contract
which requires a bunch of amount of gas
and then we can limit it.
Any other examples?

student: Yeah, you said that **validators all have to put up 32**...

Ivan: Yeah, this is basically...
Yeah, so **if I want to mine Ethereum**,
I either need to become part of a **pool**
which has 32...
I mean, if we combine ourselves and all of us
put some small amount of Ethereum
and **then we get to 32, then we can become one validator**
and **validate transactions together**.
But if I want to **do it by my own, by myself**,
I need to (buy and) **stake 32 ETH**.

student: So you can't really validate
if you don't put anything in?

Ivan: Yeah, the **idea for staking**
is that I put my money in order
**to prove that I am honest**.
Because if I validate both transactions
or ___ transactions, then I get **slashed**.

---

#### ChatGPT: "would "fraudulent transactions" be right"

Not really — **“fraudulent transactions” is not the right term here**.

---

##### ❌ Why it doesn’t fit

* “Fraudulent transactions” suggests **illegal or malicious user activity**
* Validators are **not responsible for judging whether a transaction is fraudulent**
* Ethereum doesn’t slash validators for including “bad” transactions

---

##### ✅ What actually causes slashing

Validators get slashed for **protocol violations**, such as:

* Signing **conflicting blocks**
* Attesting to **inconsistent chain states**

So the key idea is **conflict / contradiction**, not fraud.

---

##### ✔️ Best wording

* **conflicting transactions** ✅
* **conflicting blocks** (even more precise) ✅
* **double-signing** (technical term) ✅

---

##### 🧠 Bottom line

> Slashing happens because of **inconsistency**, not because of **fraud detection**.

So stick with:

> **conflicting transactions** (or even better: **conflicting blocks**)

---

**"Slashed"** means that I loose part of my 32 ETH.
I don't loose all of them,
I think there is some mechanism where I loose first half of them
and then I loose another half,
but in that case, I say, okay, I am honest,
I put my 32 Ethereum,
if I do something bad,
you are allowed to take half of my Ethereum.

student: And if you are just curious and want to validate these transactions,
can you do it independently of the validation pool?

Ivan: Probably, but yeah, then you are not included on the chain.
In order to be on chain,
so that you communicate with other nodes,
you need to put them on (ie. stake 32 ETH (and become a validator)).
If you want to do it just for fun,
you can connect to some mempool
where you get all the transactions and validate them,
but you are not part of the network
and you are doing it for fun.

student3: But if you are running your own Ethereum node,
but you are not validating,
then I guess you are like a non-mining Bitcoin node, right?

Ivan: Yeah, of course, of course.
You can run your own validator and just don't participate.
It's up to you. I mean, if you,...
most of the people do it for the money,
I assume that most of them are just participating
in order to get some money out of the validation.

### slide: ETH Denomination

Just like Bitcoin, where we have
Satoshis and Bitcoin,
in Ethereum we have also this **denomination**.
You don't need to remember them.
Usually, you use only the first one,
which is the **"wei"**
and **"Ether"**, which is the largest one.
One Ethereum is 1 with 18 zeros.
One "wei" is just one "wei".
This is, think about this, is one cent.
One euro is 100 cents, so one wei is 1,000 "Kwei",
and one Ethereum is 1 with 18 zeros (wei).

---

ChatGPT:

In Ethereum, Ether has multiple denominations:

wei → smallest unit
**gwei** → commonly used for **gas fees**
ether (ETH) → standard unit

👉 Example:

1 ETH = 1,000,000,000 gwei
1 gwei = 1,000,000,000 wei

---

In the second column,
**"unit name"**, like in the second column,
most of them are like famous scientists
or people from computer science departments
who participated either in Computer science or in Bitcoin.

---

⚠️ Important practical note

- In real usage, people mostly use:
  - **wei**
  - **gwei**
  - **ETH**
- Names like **finney** or **szabo** are:
  - mostly **historical / educational**
  - rarely used in everyday transactions

---

So, those two (**Finney** and **Szabo**) are part of
Cypherpunk, I think,
those people who started to think about privacy, privacy problems.
They get some speculation that this might be people from the founders of the bitcoin.
This was just a community,
I think, from 1994 or something like this,
where they were just discussing about
how they could **preserve privacy in the digital world**.

student3: **How much is one gas unit** equal to?

Ivan: That gas is basically,
no, so, yeah, okay.
**Gas is the liter**,
like, you take a car with just the amount of liters.

student3: It's **not a unit**.

Ivan: It's not a unit. So, basically, **depending on the transaction volume**,
right now, you calculate how much is one gas
based on the current amount of transactions.

student3: So, it's like a unit,
but it changes based on the transaction volume.

Ivan: Basically, **if there are lots of people sending transactions, one gas will cost more wei**.
If there are less people, like in some part of the day
when there is less **demand for transaction**,
then one gas unit will be less.
So, it's **literally like a petrol station where** you go,
**pay for one liter** of petrol,
but **you don't know the price of the petrol**.
It just changes depending on the day and situation.

student3: And currently, do you know about **how much one gas is?**

Ivan: It's **dynamic**.
**Every twelve seconds** ___.

---

ChatGPT:

The correct completion is:

> Every twelve seconds **it changes**.

---

✔️ Full answer

> It's dynamic. Every twelve seconds **it changes**.

---

💡 Why?

* In Ethereum, the **base fee** is adjusted:

  * **every block**
  * and a new block is produced roughly **every ~12 seconds**

👉 So gas prices are constantly updated based on network demand.

---

You can check it, but **it's very dynamic**. I will not say that right now it is this amount of money.

student3: So it changes a lot. Thank you.

student4: So, there's no way to know how much it is?

Ivan: It's the smallest unit, so, yeah, you can see,
but in terms of price,
I mean, one Satoshi was 0.0006€ at one point (March 2026), it's a very dynamic unit.
I mean, one Ethereum right now is, I don't know,
two, three thousand euro, one Bitcoin is 92 thousand dollars,
or something like this.
Yeah, basically, they have different values.
But wei is, you can see, wei is like Satoshi,
because it is the smallest unit, but it doesn't have as much value. Price is less, of course.
It's like having one cent here in Europe,
and one cent in US dollar, for example,
they have different values based on the currency exchange rate.
Yeah, that's about the Ethereum tokens.

### slide: Transactions in Ethereum (slide 1)

So, yeah.
How do we create transactions in Ethereum?
So, now you know when you create transactions,
you need to pay transaction fees.
But how, when you go one step back,
how do you create the transactions,
and what's the unique property behind those blockchain transactions?
Why do we say, ok, a transaction on blockchain is really secure?
Because, first, like on Bitcoin,
we have this idea of **public-private key pairs**.
So, this is your **private key**,
which is your secret key, you don't need to share it,
and then we have a **public key**, which is public for everyone.

### slide: Ethereum Key-Pair

And, if we go one step further,
we can see this **key pair**, how it works.
I mean, you don't need to understand this in detail,
because this is just cryptography,
but it's good to have some basic knowledge
of what is really my private key
and how do I really sign the transaction.

On Ethereum, and I think also on Bitcoin,
we use a **signature algorithm**,
which is called an **elliptic digital signature algorithm**.
And this algorithm has different kinds of **forms**:

One of those forms is this `secp256k1`, whatever,
and it looks like this (see graph on slide).
What's the idea here?
We have, by using this **geometric figure**,
we can ensure that out of our public key, we cannot go back to our private key.
I mean, this is just simply a thing,
and really is much more difficult, but you should...
And I just put this as an illustration
so that you know how the mathematics looks behind the...
what curve would create a public-private key.
The fun part of the private key is that it's this **256 bits number**,
and then out of this public key we generate basically, we need to,
now going back to the hash functions,
we need to **hash this private key** using **hashing algorithm**,
which on Ethereum we call this `keccak256`.
And then we get an **address**.
Basically, **an address is just a long number**,
but on Ethereum we say,
okay, take the **last 20 bytes** of this long number,
which we have generated, and then you have our address.
Usually it's in a hexadecimal form,
starting with `0x`, and then you have numbers
and letters from `A` to `F`.

So now when you know, okay,
this is how my private key create my public key
or my address, because there is a different address.
It's not ...
I mean, it is, but the address you use for receiving the money
is just the public key which you use for receiving the money
is just the last 20 bytes of your address.
So in essence, there is a difference,
even though people usually say it is the same.

How does it work?
I mean, **how does signing actually work?**
Why do we need a sign?
We need a sign because whenever we create a transaction,
we need to indicate that this is us, our wallet,
and we are creating this transaction.
It's not someone else who is creating our transaction.
And whenever we create a transaction,
we use our private key to sign this transaction hash
and it will produce an output which is called **"signature"**.
Basically, we use our private key to sign the transaction hash
and then we produce a signature.

- This signature has three elements, which is `r`, `s`, and `v`.
  - `r` is usually a point from this curve.
  - `s` is a secret.
    - It's a proof that we know a private key that corresponds to our address.
  - And `v` is algorithm used to decipher the whole transaction.
    - So basically, they are two types of algorithms, I think 27 and 28. So depending on which kind of deciphering algorithm we use, we just use here.

---

#### ChatGPT: "is that correct"

You’re **close**, but there are a few important corrections to make.

---

##### ✅ What’s correct

* Ethereum transactions use an **ECDSA signature** with three values: **r, s, v** ✔️
* They prove that you **own the private key** ✔️

---

##### ❌ What needs correction

###### 🔹 1. `r` is NOT “a point”

* `r` is actually a **number (integer)**
* It is **derived from a point on the elliptic curve**, but it is **not the point itself**

👉 Better:

> `r` is derived from the x-coordinate of a point on the elliptic curve

---

###### 🔹 2. `s` is NOT a “secret”

* `s` is also a **number (integer)**
* It is part of the **signature proof**, but it is **not the private key** and not secret

👉 Better:

> `s` is a value computed using the private key and proves ownership of it

---

###### 🔹 3. `v` is NOT an “algorithm”

This is the biggest mistake.

* `v` does **not represent an algorithm**
* It is a **recovery identifier**

👉 It tells you:

* which of the possible public keys is the correct one
* (since multiple keys could match r and s)

---

###### 🔹 About “27 and 23”

* It’s actually:

  * **27 or 28** (historically)
* In modern Ethereum (EIP-155), `v` also encodes the **chain ID**

---

##### ✅ Correct explanation (clean version)

* **r** → derived from elliptic curve point (integer)
* **s** → proves knowledge of the private key (integer)
* **v** → recovery ID used to reconstruct the public key

---

##### 🧠 Simple intuition

The signature `(r, s, v)` lets the network:

1. Recover your **public key**
2. Check it matches your **address**
3. Verify you **authorized the transaction**

---

##### ✔️ Final verdict

* Your structure: ✅ good
* Your details: ⚠️ partially incorrect

Biggest fix:

> ❌ v = algorithm
> ✅ v = recovery identifier

---

If you want, I can show you how `(r, s, v)` actually reconstructs the sender address step-by-step — that’s usually what makes it fully click.

---

And with this signature, which is produced again
from our private key and from the hash of the transaction,
we can prove that this user is behind the public key.
Or in other words, when I sign a transaction,
I don't use my public key, I use my private key.
Without this private key, anyone who gets my signature can directly verify that
this transaction was really signed by the owner of this private key.

So it is a little bit confusing, I agree.
But you just need to,... that's why I say you don't need
to understand this in detail, but just to have
a brief understanding, why do we need the hash,
why do we need the proof, and how does it work.

student: What is the advantage I get by adding my address
in the different hash than my public key?
So eg. in Bitcoin, you just use public key.

student3: No, you also use a hash for Bitcoin.

Ivan: Basically, this address is derived from generated public keys by hashing them,
so for the purpose of making everything easier,
you just take the last 20 bytes.

Okay.
So what kind of **security properties** do we have?
Again, this is **one-way cryptography** where you need
to just remember that private key cannot be derived from the public address or from the signature hash.
Anyone can verify it pretty fast because it's straightforward.
No secrets, I mean, in that case, no secrets means no private keys.
And **ownership** in that case is **proven mathematically using this cryptography** rather than using some kind
of centralized way like a password or whatever we use to authenticate a user.
Any questions?

### slide: Transactions in Ethereum (slide 2)

In order to understand smart contracts,
you need to know that whenever we send transactions,
those transactions will be recorded on the blockchain.
Whenever we send transactions in Ethereum,
we create a **"state change"** on the blockchain.
State change means that we basically
save some information on the blockchain.
We change the status of how it looks like now,
and this is **not reversible**.
I mean, once we've already uploaded to the blockchain,
we **cannot go back to the previous state**.
That's why we can see previous transactions,
previous directions, previous money transfers
between one wallet and another.
But whenever I **"read"** some information from the blockchain,
I **don't change the state**.
And why do I think state change is important?
Because "reading" means I don't change it,
and therefore **I don't pay any transaction fees**,
so it's free.
That means if I want to prove that my transaction
really was received, I just go to the blockchain
and I read, and by reading I don't pay any fees.
But if I send a transaction, then in that case,
I would change the state and that gives a... pay the transaction fees.
That's why it's important to differentiate
whether you change the state of the blockchain or you don't.

The **simplest transaction on Ethereum** is just like
on Bitcoin, **transferring Ethereum between two accounts**.
From one account, I transfer my tokens,
in that case, my Ethereum tokens.
This is usually the simplest.

And the more complex ones, which we will look later again,
are those which **interact with smart contracts**.
These interactions with smart contracts,
they have some activity or do some calculation
or creating a transaction.

Those are, again, **dynamic**.
We don't know how much gas we will pay,
and **those are fixed**.
I think it was **21,000 units gas**.
Any questions or no?

---

🧠 Key idea

- Simple transfer → fixed cost (21,000 gas)
- Smart contract interaction → variable cost

---

### slide: Ethereum accounts

Okay.
So, now we look into the transactions.
Now, after you know how the transaction looks like,
another very important thing to understand are the **accounts**.

Ethereum is famous because it has **two types of accounts**:
The first type of account is called an **"Externally-owned Accounts (EOA)"**,
and the second type of account is a **"smart contract account"**.
That's why a smart contract is called a smart contract account.
Usually, people call it a "smart contract" because it is shorter,
but those are basically two types of accounts.

---

#### ChatGPT: "is this correct"

You’re **very close**, but not entirely correct. Here’s a precise clarification:

---

##### ✅ What is correct

* Ethereum has **two types of accounts**:

  1. **Externally Owned Accounts (EOAs)** ✔️
  2. **Contract accounts** ✔️

---

##### ❌ What needs correction

###### 🔹 “smart contract account” (wording issue)

* The correct term is:

  > **contract account**, not “smart contract account”

* A **smart contract** is the **code** stored inside a **contract account**

👉 So:

* Account = container
* Smart contract = code inside that account

---

###### 🔹 Why people say “smart contract”

* It’s just shorthand:

  * Instead of saying “contract account running code”
  * people say “smart contract”

But technically:

> A smart contract is **not the account itself**, it’s what the account contains.

---

##### ✅ Corrected version (minimal changes)

> Ethereum has two types of accounts: Externally Owned Accounts (EOAs) and contract accounts. Smart contracts run on contract accounts, which is why people often just call them “smart contracts”.

---

##### 🧠 Simple mental model

* **EOA** → controlled by a private key (a user)
* **Contract account** → controlled by code (a smart contract)

---

###### ✔️ Final verdict

* Core idea: ✅ correct
* Terminology: ⚠️ slightly inaccurate (important for exams)

---

**Every account** has an **address**,
which, as I said, takes a **hexadecimal form**, starting with 0x,
then you get these numbers and letters from A to F.
And it doesn't matter whether it's **externally-owned or smart contract**,
it just **looks the same**,
that's why you **cannot differentiate from the address itself if it's a smart contract or externally-owned**.

A common feature is that both types of accounts,
externally-owned and smart contract, they have a **balance**.
Why do you need a balance?
This is simply for receiving money,
for receiving Ethereum tokens.

So, you see on this **picture (on slide)** here,
you can see here on the left side, the **externally-owned account**,
and then on the right side is the **smart contract account**.

**Both of them** have **nonce**,
basically this number which counts how many transactions
are executed from our wallet.
It's not the same as the amount on the block.
And they have a **balance**,
meaning that both the smart contract and the externally-owned account
can receive money, this Ethereum to their balance.

**The difference** is only those two parts here,
where the **smart contracts** have a **storage hash** and **code**.
**Code** means basically we can write code,
and by this we create a smart contract,
and this code has a **storage**,
so we can save some kind of information inside the smart contract.

And if you now think about the **externally-owned account**,
it's **just a simple wallet**.
Thus, if you already saw it in the last time,
Metamask or some other wallet.
"Externally-owned account" is just a simple wallet,
you have on your wallet,
which is controlled by public-private key.
**And Smart contract it is not.**
This is the externally-owned account,
you have a private key,
where you can create a transaction,
but for smart contract you don't have private key,
you don't control the smart contract,
you only control the code,
basically write the code,
and you define how the smart contract should perform.

student3: But **is the owner of the smart contract then the deployer?**

Ivan: Yeah, you can create the owner,
and later I will show you on the other slides,
you can create a so-called **modifiers**,
where the person who deploys the smart contract is owner,
and basically can perform different operations on that smart contract,
but by being an owner of this smart contract,
it doesn't mean that you can control the smart contract.
I mean, you are the owner,
I mean, you are the person who deployed the smart contract,
but if you don't define that in the logic of the smart contract,
that you later want to change,
or to change, for example,
the state of the smart contract, then you cannot do it.

student3: So **any control** that you want yourself to have as the owner,
**you have to have baked into it beforehand?**

Ivan: Yes, you need to define the logic.
You need to, because I can create a smart contract now,
and if I don't put myself as an owner,
everyone who sends money to the smart contract,
will just stay in the smart contract,
and they will never be able to go out of the smart contract.
So **this is like a not a bug**,
**but usually a thing you need to pay attention to when you create a smart contract**.
If with this smart contract,
we want to receive some money on our balance,
we need to somehow create a logic inside the code
ok, the owner of the wallet,
who is deploying this smart contract,
should be the owner,
and this guy should be able to withdraw money from the smart contract,
otherwise, it will be stuck forever.

So, yeah, **difference** again.
**Externally-owned account, it's just a simple wallet,**
like Metamask or like a Bitcoin wallet,
where **you have your public-private keys**.
**Smart contract, they don't use private keys, everything is controlled by the code.**
If we define in the code how to control the smart contract,
then we can do it, but not after we deployed.
Any other thing?

student3: **Where exactly does the smart contract data live** on the blockchain, in a block?

Ivan: Yeah, so that's why I said, usually these Merkle trees are a little bit different in Ethereum, so we don't have this binary, we have these **Merkle Patricia trees**, where they have a specialized storage, where you store the data and the code of the smart contract when you change the state of the... Yeah, it is a little bit confusing, **but everything, for example, the code and the storage itself, is stored inside the blockchain, not directly inside the block**.

---

#### ChatGPT: "but if it not stored inside the block, how is it stored inside the blockchain?"

Great question — this is exactly where Ethereum gets a bit unintuitive.

---

##### 🧠 Short answer

Smart contract data is **not stored directly inside blocks**, but in the **global state**, which is stored as a **Merkle Patricia Trie**, whose **root hash is stored in each block**.

---

##### 🔹 How it actually works

###### 1. 📦 Blocks do NOT store full data

A block in Ethereum contains:

* transactions
* metadata
* **state root (hash)**

👉 It does **not** store all account balances or contract storage directly.

---

###### 2. 🌳 The actual data lives in the “state”

Ethereum maintains a **global state**, which includes:

* account balances
* smart contract code
* smart contract storage

This state is stored in a:

> **Merkle Patricia Trie**

---

###### 3. 🔗 Block → State connection

Each block contains a:

* **stateRoot**

👉 This is a cryptographic hash pointing to the **entire state tree**.

So:

* Block = pointer (via hash)
* Trie = actual data

---

###### 4. 🔄 What happens on a transaction?

1. Transaction executes
2. Smart contract state changes
3. Trie updates
4. New **stateRoot** is computed
5. That root is stored in the next block

---

##### 🧠 Analogy

* Block = **table of contents**
* State trie = **actual book pages**

The block doesn’t hold the data — it holds a **fingerprint of it**.

---

##### ⚠️ Why this design?

* Efficient verification
* Cryptographic integrity
* Ability to prove any piece of data (Merkle proofs)

---

##### ✅ Final takeaway

> Smart contract data lives in the **state (Merkle Patricia Trie)**, and each block only stores a **hash (state root)** that commits to that entire state.

---

If you want, I can draw a simple diagram of block → state root → trie → storage (this is usually what makes it click instantly).

---

### org: Ivan

I think we should stop soon,
let me just check what the next slide is,
it's really good with this one,
but I think we can stop now here,
because later I will **go a little bit deeper into the private keys**,
but we can leave that for tomorrow,
not for tomorrow, for the next time,
and then I will show you, basically, **how to create the smart contract out of the wallet**.

### org: Prinz: exam registration

prof: Okay, good, yeah, Ivan I took your phone, I thought it was mine.
Yeah, I put it here.
Yeah, I checked the **registration**,
three people registered already,
so three people succeeded,
so if they are here,
Yen Ye Yan,
Junien S Telno,
and Fernandes Meiglars,
so these two guys registered.
Maybe they took different courses to study.
Probably, yeah,
so it seems to be easy because they are computer science informatics,
and I said, I could show you.
Sorry, registered for what?
For the exam.
For the exam, yeah.
Okay, so they registered.
So which study are you in?
So it seems to me that it's possible,
at least for all people,
most study something in the area of computer science,
data science,
ah, which courses are you in then?
Yeah, yeah, I'm in the master's.
Physics.
Physics.
Yes, master.
Okay.
So I have to check out at this.
What I can do is just,
instantly, I can just register you.
So are you interested to participate in that?
So what I do is I try this out,
I just register.
Okay, you are in
Bachelor's Science.
Master's Science.
Master's Science.
Okay, so I have to check how I add this particular one.
So I can't even register you
on my own power and force,
which is a little bit of a
little bit of a different way of doing that.
Data science.
Okay, okay, but then I know where to look for.
And, um,
this is somewhere that you can
have to do in the future.
I have to check how I add this.
I don't know someone I can ask for,
but no, I know where the problem is.
Okay, good.
You can still register until the beginning of January.
No, 15th of January.
15th.
You can log it out.
I can't.
The lecturers can set it immediately.
Not necessarily, I think,
and the whole performance
of what is,
that's,
that's it.
Okay, but then,
for this lecture,
chose unmeditated
until the 20th of February.
Oh.
Yeah, because I thought that you can really
register.
Oh, okay.
Maybe, maybe I just,
did the follow-up.
So, Professor, do we just wait a week
and then try to register again?
Yeah, I would send an email.
Okay.
I would send you an email.
As soon as I changed something,
I would send you an email.
Also, what's the best email to reach you at?
Because I tried sending an email to
princeatfit.famhofer.db.
And both coming up, Prince.
Yeah, that one.
Yeah.
It didn't go through.
No, it got canceled.
I don't know.
So I outlook.
So is that the one?
Wolfgang.fmh?
Wolfgang.fmh?
Yes, princeatfit.famhofer.db.
Okay.
Must be princeatfit.fmh?
Yeah, maybe it was.

### org: Prinz

Okay, good.
So, then next time,
you continue,
and then you also go
a little **programming**.
And then we also find
an exercise.
And then we have two,... I think then we have two remaining lectures.
We still have **NFTs** then.
And we have an **IPFS**.
Which is very interesting.
Okay, good.

