---
title: "Web3 and Distributed Ledger Technology - Blockchain Basics (Part 2)"
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

# Blockchain Basics (Part 2)

The hash is somehow, again, double-hatching more.
In pre-hatching, so in our block,
we could also store not just the hash of the pairwise hash,
but just hash all that.
How do we do that?
How do we do the trip of doing this pairwise hash?
Because then you can prove to someone else
that our transaction is on this block,
by just providing the hash of our transaction,
the neighborhood transaction, and the on top of our hash.
Yeah.
That's the right thing.
If you want to prove that transaction number zero
is in this here, we only need to provide this hash
and that hash.
So two hashes, otherwise we need to provide three transactions.
So we say one.
It's not that much here, but if we
take the three much farther, much further,
then it makes sense.
So we just need to give less information
to prove that particular transaction is within this particular block.
So we have less operations to do.
We need to provide less information.
And actually, I think I mentioned it last time.
It comes from communication technology.
So it also has been used to prove
that the transmission of something between two endpoints
is correct.
So it just saves information that we need to transmit.
Yeah.
OK.
So then come on.
Do something.
We went further and we discussed also that this is the case.
Then did you discuss the point that what happens if we want to pay
one bitcoin and we discussed that last time?
No.
Good.
Yeah.
Then we jump in.
Yeah.
OK.
So now, how do they actually prove that you have a bitcoin when
you want to transmit a bitcoin?
Let's imagine I want to pay you a bitcoin.
How can I show that I have a bitcoin at all?
I mean, the blockchain, in particular, bitcoin,
the theorem is a bit different.
I think bitcoin doesn't have any account statement from you.
It doesn't store what conference has one bitcoin or two
bitcoins like a normal bank account.
Your bank account stores 100 euros.
And then you can transmit 1 euro.
In bitcoin, you don't have that.
Bitcoin is actually, in general, a blockchain.
I mean, it's a ledger of transactions.
So far, we just, I mean, when we look at the diagram here,
all we start is just that we have a lot of different transactions
in our system.
Each block just stores transactions that A sent B
to send something, 10 bitcoins.
And that is all in.
And there is no account telling B has 20.
That's not part of the whole system.
That's not part of the game.
This means that if B wants to send now a bitcoin,
B wants to send a bitcoin to C, 1, you can see.
It must prove that it owns already 10 bitcoins.
And it does that by providing the information
about that particular transaction.
So you send the system, hey, here is my transaction in which
I have received 10 bitcoins.
And now I would like to send 1 bitcoin to someone else.
What happens was the remaining 9 bitcoins.
I mean, you must do some magic computation.
How do we get that?
You produce a new output that hasn't changed the address.
So what we do is then actually we put 1 bitcoin there
and we prove it by providing 10 bitcoins.
And in return, we get 9 bitcoins sent.
And I was going to say, if you forget to do that,
you end up paying a fee of 9 bitcoins.
Which is a bad state.
That can happen.
But this is not automatically by the wallet.
And again, you also have to provide some kind of transaction
fees.
We discussed it.
And I think you mentioned last time the transaction fees.
Not really depending on the amount,
but on the overall traffic.
The number of bytes that you get at the transaction.
Consist off.
And recently we did a similar tutorial.
And then somebody mentioned just, I think it was at a bank.
And they mentioned that someone sent a few Satoshis.
He wanted to send a bitcoin.
It was a large transaction.
Or 1 bitcoin is even enough.
And he wanted to provide some fees.
And he just mixed it up.
So we sent only a few Satoshis, which is the send in Bitcoin.
He sent some Satoshis.
And he provided a transaction fee of a bitcoin.
And that was just before he just mixed it.
Those of you who learn the bitcoin.
Or if you just do this programmatically,
I mean, you can use an API to do that.
Then you learn about smart contracts and so on.
You can use some programmatic transactions.
If you just mixed the two variables, it's done.
And what happens, every miner is happy.
He gets a lot of transaction fees.
So we do this regular for transactions.
So we'll quickly find the balance of this before.
Because we basically, if not immediately,
you can just find the full transaction register.
Or a full measure and understand what is the balance.
You might sum it up all.
Yeah.
Actually, and this is proven.
I mean, this is being provided.
But what?
On the other hand, the miner who is taking a transaction
is also checking that this is true.
But in this case, he doesn't need to traverse every second.
You have to find any transaction where this happened.
So that is one thing.
But what happens now if we want to send two bitcoins
that we received, one Bitcoin there
and another Bitcoin over there?
We need to send both transactions.
We need to send, well, we got this Bitcoin Bitcoin over here.
We got two Bitcoins over there.
And now I want to transfer two and a half Bitcoins.
So we need to send that.
And then we get a half of Bitcoin back.
And I think it's all done automatically somehow.
But it's something to remember that such a blockchain
is not keeping an account for you.
The blockchain is a list of transactions
that happened in the past.
And your account is actually just
the sum of all the transactions that
are being stored for your particular wallet.
So when you do a transaction, you
need to provide every transaction since the last transaction.
Because by performing the last transaction,
you get this total amount.
And then you have to sum everything up.
So you have to pay for it.
Actually, that means you have to provide the transaction that
proves that you own this particular Bitcoin.
And it hasn't been spent already.
Yeah, so you don't need to provide every single transaction,
just the ones that you need to spend the amount you want to send.
But what does stop me from spending it twice?
Like, I can provide the transaction from two months ago
where I received the Bitcoin.
And then provide that to multiple?
Maybe you know better about it.
I think it's being stored in the system.
It's being part of the transaction
that this has already been used.
But it's already spent.
Yeah, it's less than spent.
So it's being marked as spent.
Not in the block where you got it,
but in the new block where you used it.
Because the old block can't be changed.
If we got it here, and we are now doing this over here
in the block, then we can't go back and say, well, this is done.
We just need to do it here.
So the miner checks if this transaction appears later again.
And it's marked as spent.
And that's the reason why you always go from back to front.
I mean, you can't do it otherwise.
You can't change from that block to that block
because there's no link.
There is no link from here to there.
It's only a link from here to there.
Because at this point, we didn't know that there was another block.
So I provide from two months ago the block with my transaction.
And then from there on, not the block, just the transaction.
The transaction.
And from there on, you can go through all the blocks today
and just check if this transaction appears.
And you don't need to check all the transactions.
Yes.
And it's also the case.
I mean, I'm not sure how they really implemented that.
Maybe it's just as an exercise to the code Bitcoin.
I believe that all the miners, they keep local databases
that they use as cash.
Yeah.
I think they don't use just, they don't traverse all the blocks.
I mean, they use database locally in which they somehow
simulate your account.
Yeah, they have basically a list of all unspent transaction
outputs.
And then they can just check where that database runs.
Implementation level there is an account level,
but on a methodology level not.
Yes.
That's exactly it.
That's a very nice answer phrase it.
So if we wouldn't optimize the whole thing,
then we would have, then we would
be to traverse the whole blockchain.
But obviously, we optimize it.
And then they have local caches and unspent
for UXTO.
UX with UT.
So yeah, there where we saw all the others.
OK, so some other things that we asked.
OK.
Does somebody know why we should wait six transactions
before a transaction is valid?
I mean, obviously what we do is we have our transactions here.
So let's try to construct this.
So let's say that is our blocks here now.
And we have just submitted our transaction.
And this transaction has been recorded here A to B to 10.
And we got from the system back, we saw.
Later on we see this.
There's a nice user interface.
This block is now in the blockchain.
Why should we still wait six more transactions
before we really are sure that this is done?
And my example is that I would like to say,
I don't have a Porsche.
Imagine that I have a Porsche.
I want to sell it to him for a Bitcoin.
And so he provides me the Bitcoin.
And I give him the key as soon as I see, ah, it's there.
It's in the block.
He gets the key and drives away.
What could happen?
Yeah.
Like I could spend in two very different places
at the same time.
And due to the physical distance, they couldn't communicate
in time that was spent twice.
Like I could buy both types of the car at the same time.
Did I mention that it's not already done?
OK, now, that's right.
That's the case.
That's something I can do.
Another thing?
I believe there is some consensus on what
blocks and is currently in use.
So the longest one is shown all the time.
Yeah.
And this builds on that one.
Yeah, both is one.
Or maybe some miners are operating on one kind of block.
And some others are.
Yeah, we are operating on the same one.
Yeah, but after some time there is a consensus again.
Or some.
Let's try to construct something.
And so I'm selling him.
He seems to be experienced.
So I'm very unexperienced.
So I sell him my car for a bit.
What he does, what he can do is to follow it.
He, since you are friends, yeah.
So what you do is that he gives you his access.
He gives you access to his wallet.
And he submits his transaction down here.
You connect to another miner somewhere in China
at the other end of the world.
You both submit the transaction almost at the same time.
And then you just hope.
You hope that this transaction is in one block.
Let's imagine.
It's not yet there.
So this transaction, A2B, gets here in Europe.
This is German theater.
Let's assume that Bitcoin mining is German theater.
You connect to a miner somewhere
or to the network in Asia, somewhere else.
So A2B, you just hope that there is some way, not
a fast connection between these.
But you don't go to A2B or to A2C.
You're transferring somewhere else.
Maybe you're transferring yourself.
So what you hope is that this is not being recognized.
Because each miner, this one here in Europe,
or this one in Europe, this one is in Asia,
maybe because of some communication problems or so
that don't recognize that both transactions are being sent.
If we would send both transactions to that miner here,
you would immediately see that we
try to use the same bitcoins for two different transactions.
It would be rejected immediately.
You just hope they don't talk to each other fast enough.
So both take this transaction into their new block.
And then we hope that both find the new block
within the next period.
So already two assumptions, which are very strange,
but they could happen.
So what happens now is that both find the non-stake,
provide the header information, and they both submit it,
both into the network.
Which means we have two different blocks now.
This one has been mined, and this one has been mined.
Both found the right non-stake.
Both submitted it to the network.
We submit it to all the miners.
All miners receive two new blocks.
What do they do?
They just put it into their chain.
So connect it to the chain.
So now we have somehow a state where we have a blockchain
that becomes, at the end, a tree.
But only for the next period.
What happens now?
Now, so this happened in Europe.
This happened in Asia.
Let's see.
We do it in the US.
USA.
That's USA.
That's Asia.
We are in Europe.
So now we get new transactions.
And now we start building a new block.
And for this new block, we need to decide
to which previous block do we connect.
Do we connect to that block or to that block?
What could be a good rule to decide?
To which block we connect?
We don't have more money.
Similar?
Similar.
Similar.
But really, with them, we would, yeah,
more money could be something.
It's not more money.
I don't know.
Somehow it's work.
Work.
Work.
It has a little bit to do with work also.
But not all of what we have to do.
Yeah.
The one that the majority connects to.
We don't know which majority connects to at this point
directly.
But we know something else.
We know how big the block is.
So the size of the block, the length of this particular block.
So we want to continue the chain actually
at a point where has been already most transactions have
already been somehow included.
So if this, for example, has 6,000 transactions
and that has 4,000 transactions, we
will choose that one.
Because here, I'm not transaction inside.
It's more valuable.
It has a little bit to do with the value of the money.
That's in there.
And if the length is the same, then the next decision point
is that we say, we look at the nonce.
And then we say, the back of the nonce is, we choose that one.
Let's imagine we need four zeros, 0, 0, 0, 9, something.
And this one has 0, 0, 0, 5.
This is data.
It wasn't smaller.
It was more difficult to find.
Or even this has another 0 here.
Then it's even much better.
So then we would say, we connect to that one.
So the longest chain survives.
So what we do now is that we say, OK,
both have the same number of transactions.
So let's keep the game.
This is my best transaction to me.
I've sold my car for that one.
And that is you're trying to cheat me.
So what we do now is we connect to that block.
What happens now?
Next time, we would like to try the next block.
But we would like to connect to this one now.
But now what happens is that we cut that block here.
So we need to find the difference between that block
and that block, because these transactions are already done.
These transactions, there may be a few transactions in that block
that are not in this one.
Maybe that 90% is the same.
But obviously, that particular transaction here
is only in this one.
And that particular transaction here,
together with some others, were only in that block.
So they become again free.
They come back to the mempool, to this pool of transactions
that still have to be worked upon.
And they are free.
Now what we do now is that we take these transactions,
they react to include them in here.
And then we see, hold on, a spent already 10 bitcoins here.
So that transaction is no longer valid.
So it will be rejected.
And then, although I got the information
that it has already been consumed in a block,
suddenly I get the transaction, sorry.
It's not.
It has been rejected, because it has been spent already
on here.
But here's the way with my car.
And then I'm in trouble.
And that's the reason why you should wait six transactions
which is approximately 10 minutes each.
So we always get a block after 10 minutes
before you really sell something valuable.
Selling the pixel, so it doesn't matter.
But we wouldn't make such a big deal about it.
Some questions?
I did not have any understanding why I would
with the same amount of transactions choose the one
with the lower loans.
Decision.
OK.
Because the lower loans is not difficult to find
than a higher loan.
How does it affect them?
Because it doesn't affect you.
That's the miners' decision.
It's just a rule that's being implemented in the protocol.
And the second one would be why would I
wait six transactions?
There are reasons not to detect the issue.
It's just a recommendation.
It could be that again, we have this.
It's just a matter that we say, OK, after six transactions,
the whole chain will not stable.
And on the other hand, after six transactions,
even with something like a 51% attack,
it's somehow, with a 51% attack, it's somehow very difficult
really to modify six blocks back.
Yeah.
That's the point.
60% to 51% attack is that we have 51% of the mining pool
available for you so that you somehow can do something
that you somehow manipulate the previous block.
And you have so much computing power
that you can manipulate the previous block, that block,
and the next one.
And you find much faster these blocks than the others
find the next one.
So you have so much computing power
that you can do some reconflictation of previous blocks.
And suddenly, you send it to the whole network,
hey, this is now the longest chain.
So imagine that we have something like this here.
So we now, that's the normal chain that's going on.
And they are computing every 10 minutes, 10 minutes, 10
minutes.
And we are now, we just bought so much hardware
that we become much faster.
So what we do now is that we reinsert transactions over here.
We modify transactions over here.
And what we do now is, what we have to do
is that we have to recompute the hash of that one
because it changed.
So we recompute that one.
Then we do very quickly that one, that one, and that one.
And define that, maybe, for example, within five, five,
five minutes.
So let's say, you know that's already 50 minutes.
So this 20 minutes.
And then we submit that chain into the network.
And suddenly, this chain becomes longer than that chain.
So this means the next block of all the other miners
who are in good assumption that everything is fine
connect their blocks to this one.
And this means that we did some kind of modifications over here.
And that's this 51% attack.
Could also be, and that's a similar attack,
that we somehow try to do some somehow
disconnecting them off by doing some kind of, how do you call it,
where you send too many requests to WebSolve or Denial of Service
attack.
Yeah, you do some kind of Denial of Service attacks to them.
So that they are just being cut off from the network for some time.
In the meantime, you do some kind of re-chaining the whole thing.
And suddenly, they come back and then they see,
oh, that's not how the account stayed.
So we have to connect to that one.
And here you did some tricks.
The point is the 51% attack is real.
It can happen.
But if this happens, what would happen in the same thing,
everybody would lose trust in Bitfarm.
So you earn a lot of money by this versus.
So the one question is, I was going to ask more that, OK,
you have first a lot of computing power to do this.
And then you have to do it subsequently three times
for you to beat the other person that did it one time.
And it didn't wait 10 minutes, for example.
How doable is this?
How much computing power is it even possible?
I mean, one computer won't do it.
No, no, you would need the whole computing center to do that.
And we will have a look at Mempool.
Then we see who is company mining.
And then let's move a little bit.
Let's wait a little bit.
Let's delay the answer.
Yeah?
I can.
And the one that I'm going to move on,
can I come to the end of the table?
I can get all the data and I can get the correct one.
You don't claim it's the correct one.
You just submit the whole blocks.
And the blocks are correct in themselves.
You just manipulate it one transaction in there.
So what you did is that I transferred one Bitcoin
to you.
And you just insert here not one Bitcoin,
but maybe 10 however.
Maybe even I have 10.
Maybe then I got 10.
If everything is correct.
I had 10 and you're just stealing the 10 Bitcoin instead of one.
And then you're just computing all the different blocks.
So the blocks in themselves are correct.
They have the right non-stay, change to each other.
You just manipulate one transaction
by stealing real money.
And if this is that long lot, then the other ones,
then it's being accepted.
You don't pretend it's correct.
It is correct.
It's a correct change.
But you just manipulated one of the transactions.
But they wouldn't be able to steal Bitcoin that hasn't been spent.
What they could do is they could rearrange transactions
that have already occurred.
Because it would require validating signatures instead of that.
So it's not easy to do.
But it's just something could happen.
And when you talk to people about blockchain,
they come along, ah, 51% of the tech.
Blockchain is insecure.
Because we have the 51% of it.
We have similar techs.
I've heard that a lot of Bitcoins have been stolen.
We discussed later on a little bit
how you really can steal Bitcoins, but not using that.
OK.
So something else, what we can also say,
is that some numbers, so in the end,
we will have a maximum number of Bitcoins,
which is 21 million Bitcoins.
We will not have more than 21 million Bitcoins.
That's in the protocol.
And I told you already that we have this Bitcoin halving.
So approximately every 210,000 blocks,
we do some kind of halving.
Which means that the incentive is coming here 3.1.
And then it will be 1.56 something.
And this takes place in April last year.
We have the last halving.
And in 2140, the last Bitcoin will be mine.
And then there's no more Bitcoins.
There will be no more new Bitcoins.
There will be 21 million Bitcoins.
And this is now the point that everybody says,
and that's the reason why Bitcoin will increase in value.
Because there will be no more Bitcoins,
but we need more money.
This means that the individual Bitcoin
will become more valuable.
We need to learn more, I don't know.
But that's the point.
And something interesting that I also just learned last week
is the following.
So which means that we have one period in time
where we got 50 Bitcoins as an incentive
for our mining fees.
That was just at the beginning, the first years.
So it took about four years.
Then it was decreased to a period where we got 25.
Then it was decreased to a period of 12.5, 6.5, and the rest.
And just a very interesting thing is the following.
That in this time, 50% of all Bitcoins have been produced.
So during the time that we got 50 Bitcoins per block
as an incentive, we produced 10,500,000 Bitcoins.
50%.
Then in the next time, when we got 25 Bitcoins as an incentive,
we produced 25% of the total number.
So it's some kind of what you call it anecdotal knowledge,
somehow, which is quite funny, which you can tell in the background.
And that's how it works.
So at the moment, the current period of five years,
we actually produced only 3.1% of the whole sum of Bitcoins,
which means that most of the Bitcoins have already been produced.
There will not be that much coming in the end.
So that's the time that we go to this web page here.
Let's see.
That's Mempool.
Mempool is a web page that is here.
It's come in live.
And what we see here are the Bitcoin blocks that have been mined.
On the right-hand side, we have those that have been mined.
And you see the amount of time that has been passed since this time.
So 50 minutes ago, 46 minutes ago, 42, 30, 60, 7 minutes ago.
So you see, it's not always 10 minutes.
It can be a little bit more.
It can be a little bit less than an average.
We get it.
So here we see the average block time, 10.2 minutes.
So what else can we see?
So what we can do is, we see the miner.
That's also interesting.
So we see that there are mining pools here.
F2P pool got this block.
Mine this block.
F2P pool mined that block.
So you see, we have some pools who are really powerful.
It's not that it's a completely equal distribution who's getting the mining fees.
And who's moving the block.
We also see the number of transactions in the block.
Let's see.
We just take this block here.
And we click on this.
We should see some more information about that.
We should see.
Sorry?
Yeah, here we go.
So that's this block number.
And then we see some additional information about it.
So we see.
Okay.
So, no.
Tricking the side of you.
Yeah, that's just like to scroll down for a second.
Yeah.
Now we see the different transactions in there.
I was looking for the summary of that.
That's what we got.
You see, that's the, we got 3,360 transactions in there.
We got the total fees that we earned.
0.02.7.
That's what it was.
And here you see all the transactions that have been spent in there.
Yeah.
What else can we see?
That first transaction is the coin-based transaction.
So that's the block reward.
That's the block reward.
3.25.
Also all the fees.
Yeah.
So there you see what they earned.
And this is how we check all the different transactions that have been done.
And in the bottom left of every transaction, you can see the fee rate that they're paying.
So in Satoshi's per virtual rights.
So then you can see the total fee and then the fee rate.
Yeah.
That was five dollars.
Key.
And what they transferred was actually 0.0.
Oh, they earned 1.5 bitcoins.
Yeah.
That's the fees, isn't it?
No.
So that, so you consume one previous output and produce two new outputs.
So what did they pay?
They paid five dollars for a transaction of one bitcoin.
Yeah.
They paid five dollars for a transaction of two bitcoins.
So there's quite something going on here.
People are moving 200,000 euros or what?
Let's see.
Let's look for a very small transaction.
What are the little ones on the right inside?
The only, the top one is what's actually, I have the bottom one probably goes back to your own wallet.
The left is what is being spent.
And then on the right are the new outputs that they're producing.
So that's the fee and that's the...
No, those are two outputs.
Then the fee is the difference between what could go to you and the bottom one could go back to me.
Yeah, I can use it again.
Yes.
Can you see?
What do you see?
What you can see is usually what wallets do is they produce a new address for your change to go to.
To increase your privacy.
That way someone is looking at the blockchain.
They don't know which is your change and which is the actual thing that you want to do.
Or else, yeah, so what you could guess is somehow,
he provided a transaction of one Bitcoin, that's what he got some time ago.
He's sending 0.0007, not that much, to a new address, for example, to you.
And he's getting 1.02 back.
But he's sending that to a different address, which is also owned by him.
Just to somehow say, just to avoid that.
If I know you're good, if I know you're wanted address, I can identify how rich you are.
And if you are very rich, I can try somehow to cheat you, to knock you down, to make you drunk, whatever, in order to get your Bitcoin.
Yeah, that's the reason.
Bitcoin is transparent.
It's not about what people have on the ledger.
To avoid that, you somehow do some kind of mixing and using a lot of different addresses.
A lot of different account numbers, like in a bank.
Yeah, if I wanted to ask, I could track, if I really wanted to.
Can I point at something?
Yeah, yeah, yeah, yeah.
Oh, of course.
Yeah, so here the red means it's been consumed.
So because this is the one that's going into the transaction.
And then here, when you see green, it means that that transaction hasn't been spent yet.
And then if it's red, it means that this output has actually been already spent in another transaction.
Yeah.
Hasn't been spent means that this is not yet the mine somewhere.
So this one, so for example, in this transaction, it spends one input, which is why it's going red.
Because it's being spent in this transaction.
And then the two outputs that it produces, this one has already been spent for 7 Bitcoin because it's red.
And then this one hasn't been spent yet.
So the bottom address could use that transaction to create a new transaction.
Exactly.
So when they want to spend this Bitcoin, you will see this one on the left side.
I think this should be summed up to that one.
Yes.
And the difference between, so what you see here, the fee, the fee is literally just the difference between the inputs and the outputs.
That's all the fee is.
So if you make a mistake and you forget to add an output, then I will just end up as a fee and the miner will get it.
And I think the highest was at 86 Bitcoin.
Someone made a mistake a long time ago and 86 Bitcoin went to the miner.
Just because they made a mistake.
So if you're looking for one transaction, it's really just free addresses and free values for Bitcoin.
Well, as many as you want.
So if you scroll down there, you can find another one with, usually you can, okay, like here for example, you can have as many inputs as you want.
So all you have to do is when you make a Bitcoin transaction, you reference UTXOs, which are unspent transaction outputs as inputs.
And then you can generate as many outputs as you want.
So the means the closer the transactions you're used to prove that amount of Bitcoin are to what are you spending the lower your fee is, right?
The fee only depends on one, how much you choose to pay, but also the size of the bytes of the transaction.
So like here for example, you can see the fee rate, which is supposedly for virtual bytes, the byte basically.
And then that's the total fee.
And that only depends on the size of your transaction.
And the size of the transaction is just how many inputs you're referencing and how many outputs you'll produce at that.
Like if you just have one input and one output, but you're transferring 50 Bitcoin, you still only need to pay a tiny fee.
And then the expense in this block, you just reference the quantity with the red arrow to someone else and this with the green arrow, the credit and fee.
Yeah, so this one has already been spent.
So if you click on that one, you'll actually see where that one has gone on this address.
And then this one is still unspent.
So then this address you see here still has this transaction output sent to it and it hasn't yet been spent.
Yeah, we do this by the way.
I think this is also quite nice because if you sum this up, then you get to 2.26 and this is what you actually send.
So that's a very nice example where someone is taking two transactions in which he got some money, is bundling them in order to send this to the universe.
Yeah, he's not getting anything back.
And here he is sending something.
He's got a 0.3, 0.03 and he's sending 0.02 to someone and he's getting that back.
So he still owns that one.
And because it's pseudonymous, we don't know actually which is this person's address.
Either one could be the change.
I just clicked on this one.
Yeah, so let me see.
So it's not an address.
He got that one.
So this particular is now lucky he got...
So then you can see the balance of just the sum of all transaction outputs.
If you go to the address with a lot of outputs, the balance you see is just the sum of all the unspent outputs that they have.
So, and this hash...
These aren't big transactions.
Yeah, that's the highest thing and then lowest thing.
Transactions.
And for transactions to get accepted currently you need to pay about once a third sheet per file.
If not, miners will choose not to relay your transaction because you don't pay enough.
You know the difference between the expected block and actual block?
The expected block is so a miner just running some default Bitcoin Core policy accepting transactions.
This is what you would expect and then the actual block to get is what the miner has actually chosen to do.
Because a miner can choose any transactions they want in the block.
But what happens is they obviously pick the ones that pay the highest fees because then they get the biggest block reward.
So if I have a miner I can mine my own transaction for free?
Yeah, yeah.
Yeah, and actually there are still transactions that get sent that pay zero transaction fees
but that's usually someone who's literally a friend with a mining board.
Because if you just submit the transaction, paying zero fees to the network, they won't agree with it.
In the early days that was the norm but now that mining is becoming economic you need to pay a certain amount of fee.
And actually if you want one can go to the very first block and you can see if you go to the top right you can put zero.
If you search for zero it will click on yep.
So here you can see this is literally the very first Bitcoin block with Satoshi paying himself the first block reward.
And then as you go on you can see that there's just one transaction for block because nothing had, no one was using Bitcoin back then.
And then these are all Satoshi's wallets which he still hasn't, that Bitcoin hasn't been spent yet.
So there's about a million Bitcoin just locked away in all these wallets.
So you go down here you see it's not spent yet, it's still green.
So those 50 Bitcoin have not been spent yet.
So you can imagine that there are some people now just watching this particular address.
And if suddenly this address is being used for transaction everybody will say hey Satoshi won't cost the issues.
Or quantum computers.
Like realistically can he even spend it?
Like if he goes to a pit 3 and 5, he starts from red light.
It's really not.
Yeah, it's not.
I mean it's valid this way.
It's just people say either he's dead or he has chosen not to spend all of this to show restraint or trust in the system.
And if you do see the coins moving one day either he's working up or quantum computers have become reality.
Because here these were the earlier type of addresses and they're just the raw public key on the chain.
Whereas recently the addresses are actually hashes of the public key.
So the public key is not revealed until you spend it.
Whereas here the address is the public key.
So the future of quantum computers would be able to steal this Bitcoin.
But that's probably still the long way up.
But is that a Bitcoin?
So he could spend it if he wanted to.
But if he could find that, I mean what we need to do is you can start working on that.
You need to use the algorithm that produces public and private keys.
And you need to produce as many key pairs as possible until you've got a public key that is equivalent to that one.
And then he becomes a project.
Yeah, and that would take longer than the age of the university.
Or just good luck.
Very good luck.
And if you scroll down you can actually see when you have more than three inputs it shows this but they're so small you can't see them.
It shows all the unspent transaction outputs that a particular address has.
But they're so small there was some dune that said that's what she, more Bitcoin, gave.
That's what you mean by nothing.
No.
That's a very nice tweet.
And you can also see that the hash of the block is much, much larger than nowadays because back then it was extremely easy to mine Bitcoin.
There are like 10 or 8 zeros.
There was some, this is the hash of the block.
There were also sometimes people who could see the nones.
Yeah.
That's not the nones.
I don't think you can see the nones.
No, this was the side.
So in the URL is the hash of the block.
Yeah, when you go to a block.
So you can see all the leading zeros currently being used by the block.
But also what they vary is they vary the extra nones, which is actually inside the coinbase transaction.
Because the nones is full bytes, so it only allows about 4 billion different values.
So in order to get more nones, they started inserting a nones into this data bit of the coinbase transaction.
And then that provides a different block hash every time you vary this extra nones in here.
So then that's how they managed to get these little hashes.
Because if you could just vary the nones, then you would just be able to get 4 billion different hashes.
Which would not be enough to get this huge amount of leading zeros.
So they are somehow extending the solution space.
Yeah, but they're not.
Exactly.
They made from the hash space there.
And then these outputs are outputs that you can produce that spend creating zero Bitcoin outputs.
And you use them to just add data to the blockchain.
Yeah.
So on a Friday evening when you don't know what to do, just browse the Bitcoin blockchain.
It's a very cool website.
Yeah, thank you very much.
So with the information from, if I spend enough gas fee, I could write information on the blockchain.
Yeah, you can, the more data you put into those OP return statements, the bigger your transaction is going to be in the higher fee you'll have to pay.
But there's actually, there was a recent war in Bitcoin about whether to increase the limit of this OP return.
Because if you make the limit, if you remove the limit, then you could just put arbitrary data on the blockchain.
Which most people don't want because then you could have terabytes of blockchain size where people just putting images and anything onto the blockchain.
But you could.
Like it's consensus valid.
Yeah.
That's actually the point.
I mean, we come to a tutorial and when you learn about writing smart contracts on a theory, this is where you can put a lot of information, more information on the blockchain.
And then you have to pay more gas fees and not transaction fees to the video.
With Bitcoin, Bitcoin is not meant to start information because it's transaction oriented.
So it's just a transaction.
It's not meant really to start programs or any kind of data.
Data we start on the theory or any of the other kind of chains.
Great. Yeah.
Cool.
I also learned something.
That's always great.
So there is something that we need to distinguish now when we discuss again the differences between Bitcoin, blockchain, distributed ledgers.
And we're sure you don't have this.
But in general, I mean, actually you could argue that actually this is the database, although we can't store much information in there, but it's kind of database.
It's storing transactions.
Where is the big difference?
And the big difference is the following.
I mean, also a bank is running a distributed database.
I mean, the Spark has the Spark has the archon.
They don't have just one database server somewhere in the seller.
They have several database servers for redundancy, performance issues and a lot of things.
So they're running very complex oracle servers or whatever.
Maybe even a old cobalt database.
But I just recently learned that the lever is still running eight big mainframes with cobalt.
And these are things we do all the transactions.
You buy something at the lever.
This transaction is being handled by the mainframe on which cobalt is running.
Cobalt is an old programming language.
You become rich knowing it in the meantime because there are not many people around to know how to do this.
But that's actually at the boundary.
And these nodes, they trust each other.
All the nodes of the database, they trust each other more or less because they belong to the same organizational boundary.
And when you want to access that, you need to get access to the whole thing.
If you access the distributed database, you access it with your user ID and password and that's it.
And then you are in.
And then the whole thing, there is more or less transparency.
The difference now with blockchain is that we have again the servers, we have again the nodes,
but they don't trust each other somehow.
So what you do is you have to trust the boundary in here, but by the protocols that we run,
the consensus, the proof of work and all the rest, we can trust each other.
But they are all organized in different organizations.
As I said, Asia, whatever, all the different mining pools.
And that's the really big philosophical and methodological difference between the distributed database and the blockchain.
The nodes, they don't trust each other.
They are in different boundaries.
And that they trust each other because of the protocol.
With the distributed database, you don't need this kind of interlinking of data and the rest.
They are all trusted.
They all access the same table, whatever, the same file system in the end, however they are organized.
So that's the big philosophical difference.
At the point is now that with Bitcoin, although it's the big representative of the blockchains at all,
the point is that it's somehow boring.
I mean, yeah, we can do some transactions.
Nothing more.
We can't do any computation.
Nothing.
We can store data, just a few bytes, then we have to create one.
What did you say?
One Satoshi per byte?
What's Satoshi per byte?
One Satoshi per byte.
Yeah, so if you want to put a sentence into there, you can imagine how expensive that's going to be.
And if you want to store a picture, it becomes even more expensive, so it's not possible.
But then there was someone else coming around, Vitalik Buterin.
And Vitalik Buterin, he said, well, there should be no difference.
There is probably no difference between storing a transaction on the blockchain and a program.
So the idea was just, can we store a little bit of additional kind of programs on the blockchain?
Which means that not only the transaction becomes immutable, also the program becomes immutable.
So it means we can write immutable programs on the blockchain.
In the first instance, OK, we can't change the program anymore.
Good.
But this also means we can make a contract, so we can make a deal contract.
And then we qualify this contract and we put it on the blockchain.
And that means that from then on, nobody can change the contract anymore.
Now we do the same.
When you get a contract sometimes, but sometimes on paper, what they do is they just take the paper
and they turn the top of the paper and then they do some kind of stamp out of there.
So that you see when you somehow take pages out, you immediately see that something is wrong.
And this is now with this kind of smart contracts.
So the idea was we store programs on the blockchain.
One thing.
So we need more data space, obviously.
The second thing is we need somehow an ability that the nodes can understand the code that we put into these smart contracts.
So the nodes are not just storing transactions.
The nodes can execute the code.
And that leads them to smart contracts.
And a smart contract has nothing to do with any cleverness.
It's not artificial intelligence.
It's not smart.
It's nothing.
It's typical US way of describing something, a smart contract.
And that's often the misunderstanding that people say, ah, these contracts are smart or whatever.
They are not.
They are extremely simple.
The second thing is they are not contracts at all.
Contracts we do among humans out of the blockchain.
So we make a contract and then we put it into code and then we put it on the blockchain.
So the very simple thing is that we say, okay, whenever we have a condition that the temperature of our meat in a particular container is about 80 degrees, then we just raise the temperature alert.
That's a condition that we agree upon.
Or when you buy a ticket with the Wunderstein, then if the trade is late more than 60 minutes, you get 25% discount.
I'm not sure if this is possible to do in the app in the meantime.
Last time I used it ahead of film and it's full.
But let's not think in blockchain terms.
In blockchain terms, we made a contract with the Wunderstein.
We bought a ticket and this ticket includes that I can go on the train.
On the other hand, part of the contract is that I get a discount of 25% if the train is late.
This could be immediately detected when the train arrives at the train station and then the smart contract pays me 25% immediately.
And that's the thing where a lot of smart contracts are going to.
I mean, that was the idea of the immediacy of an event and the payment.
You deliver something, you get paid immediately.
And that's the reason why there were beginning a lot of ideas of, well, you ask me, can you take this parcel back to Cologne?
And I say, yes, I can take this parcel back to Cologne.
And as soon as the parcel is the GPS receiver detect that it's in Cologne, I get paid.
So delivery payment.
And the machine is producing something, the machine is saying, well, I produced that payment immediately.
All that can be done.
And that's the very simple idea of smart contracts.
There will be a click of mine, you will tell you how to program smart contracts.
So I will announce that.
So then you could also bring your laptop and then you can do somewhere in the sandbox.
You can play around with some kind of smart contracts and also be exercised then.
I was thinking about this, like, okay, delay of 60 minutes, but then DB puts the delay, not you yourself.
But let's say there's something that depends on objectives, you know?
You can, if you know the person who's putting in that the objectives were checked out, you can still receive the money even though you didn't do anything.
Right. Exactly.
And this gives us brings us to the next term, which is the Oracle.
Yeah, the Oracle is the thing in blockchain terms that tells the smart contract about something that happens out of the blockchain.
Because neither this is being known by the smart contract nor that.
And that is then the Oracle.
We will discuss later on.
If you like, you can look at the UMA blockchain network.
It's an Oracle network where you can set up particular kind of oracles.
And then you can say, well, I need an Oracle that tells me if today in Aachen the temperature goes above 10 degrees,
because you need it for some kind of betting.
Yeah, and then somebody can say, well, today is below 10 degrees.
And if he's lying, he has to pay some punishments.
Now, we talk about this later on.
There are already blockchains that are being used to regulate oracles.
We need that when we talk about prediction markets, about a putting market.
Yeah, and afterwards, maybe we do it for Christmas and then you can do some gambling or a person's.
Because it's actually it's called a prediction market, but it's gambling.
It's betting, otherwise.
So is somebody already dealing around with the putting market?
No, even not you.
No.
Okay, then you learn something moving for the market.
I was just wondering what are these oracles that are not set for authority?
Could be also potential authorities.
The Oracle is it...let me say, I'll give you an example.
Normally, I explain this kind of smart contract in a way that I use football.
So I'm a supporter of Cologne.
Somebody supporting by a mission or...
Normally you find out.
So that's it?
Liverpool is good.
I think we play Liverpool in two or three weeks before Christmas.
Yeah, congratulations on the win last day, yesterday.
I have a sports group and everybody Liverpool's wins is sending logo and then Cologne fans.
So let's imagine Liverpool plays Cologne, we do a bet.
So I bet 10 euros and you bet 10 euros.
Liverpool wins or puts a draw and since I'm the underdog, yeah, if Cologne wins, I get also 10 euros.
What do we do with that? This 10 euros.
I mean, you are completely unreliable.
I'm not trusting you that after Christmas you pay me my 10 euros because Cologne wins, obviously.
What happens now? We go to him, we give him 20 euros.
He's taking the money and runs away.
So he must be trusted.
He is now the new intermediary. He is now our new notary.
Obviously what he takes is he takes the 20 euros and if I win, he pays back only 18.
He's got two euros for me.
I'm the betting pool. I'm your trust provider.
Yeah, therefore I need two euros as a team.
Now we are clever. We make a smart contract.
We post, transmit 10 euros into the smart contract and then we program the smart contract in a way that we say, okay,
on the evening of the game, watch the kicker website and when the kicker website reports the result,
then do either a transaction to you or to me.
We do this programming once after you did the smart contract lecture and then it can't be changed.
Even I, as a professor, cannot change the contract in the lecture and you can't do it.
And then we are sure that we get the money.
But there are some, we have to be careful.
If we just do something like whenever Cologne plays Liverpool's, then the smart contract will always fire when they play.
Problematic.
So maybe you become rich because in the end, Liverpool's a little smaller than Cologne.
On the other hand, maybe you have to fill in the 10 something before.
Otherwise you can't do a transaction.
But how would we somehow, if we say we can't modify the smart contract, what's this idea to cheat me?
If Cologne wins.
What would you do?
Cologne is winning.
Six o'clock Cologne wins to one.
90 minutes.
90 minutes.
I can, yeah.
He sends all his money to some other artist.
No, it's blocked by the digital contract.
I can't do that.
You can do programming this way, yeah?
He goes to the trust provider, gives them some extra money and tells them to pay him out.
Yeah, so he tries to convince kicker.
Yeah, because we said, well, have a look at the kicker website and check the result from there.
So he's just trying to negotiate with the kicker website that always the Boone's League, the A-Website,
that they put in the result at 12 o'clock at night because our smart contract looks at 12 o'clock
to get the result.
Could be done.
Much easier.
He just hacks the kicker website.
Yeah, therefore, and this gives us a lot of indications.
When you think about a blockchain use case, think about it from the beginning to the end.
Just by making a smart contract that's not secure.
When we involve an oracle, you have to make sure that the trust provider is trusted.
Or we do something like with this Fuma network that we say we need some kind of decentralized trust provider
who's providing the result.
If this trust provider is providing the wrong result, he's punished in a way that is not worth doing it.
So maybe you have to put in 100 bitcoins or whatever.
Let's say 100 euros to provide the result about it.
And this must be much higher than the back itself.
So it's not worth doing it because you will lose it.
If you provide the wrong result, it will be seen the day after it will be seen and then you are punished.
So what is smart contract keeps our 10 euros we will have in the smart contract picture?
Yeah, yeah, yeah.
It's not that complicated to learn how to program Solidity.
Solidity is the programming language to write contracts on Ethereum.
And Ethereum is the blockchain that introduced smart contracts.
That Vitalik Puto invented Ethereum and that's Solidity.
It's a little bit like JavaScript.
Similar to JavaScript.
You will also see then how to do that by, if you like, you can have a look already at OpenZepelin.
OpenZepelin like the lecture.
OpenZepelin, there are a lot of smart contracts that you can use as templates.
So it's a library like StackUp.Lob.
Now StackUp.Lob is not a library but GitHub.
It's a GitHub for smart contracts.
Okay, so yeah.
But like when the runtime, because for the runtime, is somehow in the guest room?
That's in the guest room.
Yeah.
Because otherwise I can write a smart contract.
You could write a smart contract that runs endlessly.
Yeah.
And then you block the blockchain.
Yeah.
Then you kill the Ethereum.
It's a denial of service attack on Ethereum.
It's not possible because for the execution of the smart contract, you would need to provide a guest team.
Yeah.
Either you somehow upload some money to the smart contract so that it's going to run in
or we're sending the result you provide some kind of guest team such that the smart contract can execute the code internally,
which is if the result is larger or equal than this, do that transaction, else do that transaction.
I mean, it's not much.
It's a condition.
How much that is going to be, the smart contract will somehow tell you.
You can ask the smart contract, you can ask the blockchain, particularly ask them why now, how much it will be.
And then you need to provide the guest team.
And if you are not providing enough guest team, the smart contract will be back in.
We just had it.
We had a big event tomorrow and we are showing something there.
And this morning I was testing around and suddenly nothing works.
So we had to upload the wallet with some more money and then it started working again.
Because there was not enough money there to execute.
That's just a whole trick.
That's the trick to avoid that someone is just doing the network service and the blockchain will cost you endless money to do that,
to program it internally.
I don't think people criticize him for doing the JavaScript.
He should have used something else.
He should have used something else.
I know there are some other programming, other blockchain users who use Rust.
They use Rust or something else.
I don't know what you are using.
Because you don't know JavaScript as well.
You don't know any of the programming you are using in JavaScript.
So it's JavaScript with some extensions.
You need some extensions.
We will see about that in a little bit.
So from an abstract level, we start program code as executable scripts
in a transaction and they execute it in the blockchain network.
And that can do you ecosystem.
We suddenly have a world computer.
That was the first idea.
We have a world computer that is running in a decentralized way that is executing small programs.
It's a big cloud computing system.
You can just submit a small program and whenever you send it to transaction it will do something.
Like executing bets or transmitting something.
Or you can say whenever the temperature is above 10 degrees I would like to get some money.
Where we get into the area of insurances.
Whenever the temperature is for 10 days more than 30 degrees I would like to get an insurance deck.
So you can develop suddenly some kind of automatic insurance.
You mentioned the gas fee.
What is the transaction fee?
Transaction fee is the transaction fee to execute the transaction.
And the gas fee is the fee that you need to run the smart contract.
Very often it's the same.
It's not very much differentiated when you submit something to the area.
But let's say maybe we could differentiate it by saying if you want to transmit Ether,
which is the coin on the Ethereum blockchain.
When you want to transmit Ether then you can use the activities but when you want to execute a smart contract you use the NSE.
But this comes also later on.
And so the gas fee is kind of like gas.
So the longer the contract takes the more.
You just compare it. You fire an engine.
How long is the limit in code that I could upload?
I mean normally smart contracts.
Like could I say that you get on Bitcoin if you provide a program that meets all the test cases that I provided.
So do you like to do some evaluation of the question or something like that?
Like putting it on Ethereum blockchain.
So I would recommend another computing system.
I mean this is not Amazon Web Services.
You have a Google Cloud.
So it's not being meant to do computation intensive things.
This is what you always do outside.
Okay so I think you really get a new ecosystem with autonomous transactions.
And that really the basis for IT solutions.
You have internet things. You have things that do something.
And then they actually do something.
Just as a hint, you can exercise some of these kind of use cases.
But let's say we have an automatic vacuum cleaner.
The automatic vacuum cleaner could somehow charge you for cleaning your floor.
It's cleaning the floor.
And then it's reporting to the blockchain.
Hey I cleaned 21 square meters.
And one square meter is a particular token.
And the second worst let's say is a coin or whatever.
And then it's being paid.
Or you charge over.
You somehow provide this vacuum cleaner some money.
And then it's just cleaning your house.
And if it's empty, it stops.
So that's possible.
But you have to be careful.
It's irreversible.
Once you implemented it on the blockchain, it's running.
Always. Endless. Ever.
Whenever you initiate it, it will move on.
And if you made an error, that you submitted a smart contract for your vacuum cleaner.
And you mistyped the address to where the transaction should go.
The money should go.
It's gone. You can't change it.
Therefore, as we've also discussed in this video,
you can include not really deck doors, but you can pause and translate the smart contract or something similar.
So that you say, well, it should only be one if it's activated.
And you can activate it because you have to access rights.
So how do people edit smart contracts?
Or if they can't, do they just have to put a new version out?
Yes, you have to put a new version.
Every smart contract, when you deploy it, so when you install it on the blockchain,
gets a smart contract address.
And if we deploy our very smart contract, it becomes an address.
And the oracle needs to send the result to that particular address.
If we agree that we made a mistake, you know, and we say, OK, let's forget about that smart contract.
Let's deploy the new one that's corrected.
Then we need to tell the oracle submit the results to that smart contract, to the new one.
That's the only way. We can't modify the old one.
It's impossible.
Therefore, there are some tricks, but that's coming up that you somehow do some kind of an interface smart contract.
So you have an interface smart contract that provides the function calls.
And that smart contract is then calling another one.
And you always can change the address that it's calling.
So when we see, ah, there is a mistake in our smart contract.
So we just changed the address in the smart contract, so it's no longer calling the wrong one, but the right one.
And only one person can change the address that it's calling.
Because we only have the right to do that.
But the calling one is not the change, right?
Both are on the chain.
But how can you automate one?
You don't alternate it.
I'm jumping a bit back and forth, but that's okay.
What I do is somewhere on the blockchain, I have a smart contract.
Let's say this is my contract and this has a message read and a message write and a message check.
And we also have set SC address.
So what you can do, you will learn that each method can require that only a certain user is accessing it.
So when you want to access it, you must have a particular identity to it, like user ID and password.
So now we have another smart contract and that's now the address smart contract number one.
And we have smart contract number zero. That was the first one.
And that had some trouble.
At the beginning, we said call smart contract number zero.
And now we see, and this implements write and check and read.
And if we get a function call here, we just forward it to that particular smart contract because of that address.
Then we see that this smart contract is buggy.
Then we say, okay, set smart contract address one.
So we say one, we just repeat it. We can't delete that, but we say now this is not the active one.
That's the one to call and then it's forwarding everything to this.
And if you see, if you want to do some kind of improvement, we develop smart contract number two.
And then we just replace that one and say to smart contract number two.
So this method can be called with arguments.
Yes, yes, yes.
And the arguments can then provide it from outside.
Somehow you can consider this as a class that has provides methods with other.
Yeah.
Is it possible to make a contract this way?
Yes, you could expire it.
I'm not sure. Well, that's interesting.
It would be interesting to check this.
I think it should have access to a date.
Yeah, and then you could say, actually, only if the current date is below number eight.
Let's check this.
There is, when you call that particular method here, you need, it can require that this can only be done by the owner.
Yeah. So you can set owner as a requirement.
So you say owner and you say new smart contract number.
Yeah.
This means that only the owner can execute that particular method.
And the owner is the one who deployed it.
So if you deployed it, you know, you deploy it with your, when you deploy it, you somehow look into the blockchain and you do that by using your wallet.
You come to that just in a second.
So you do it with your wallet by providing your identity.
Yeah. And then when you want to execute this transaction here, that you say change the smart contract number to a new one, again, you have to identify yourself as the owner.
And when you come to that, the function will not execute.
It will be executed.
It will be executed.
Yeah.
Yeah.
Yeah.
Right. And therefore it's always a good thing if somebody is setting you something based on smart contract, check the code.
If there isn't any backdoor in the code that allows them to change the code.
Yeah.
Maybe, yeah, we probably won't reach that.
We will see, I will not reach out to a smart contract later on maybe and then you see it's actually working like that.
Yeah.
We can deploy another contract.
You can deploy another contract.
You can deploy another contract.
Yeah. And but then you would need to tell all the users of that smart contract that you have deployed to do smart contract that they have to change the address.
It's like, don't go to this door, go to that door.
If someone is still going to this door.
Yeah. So it's something, it's, I mean, not change this.
And that's maybe the reason why a lot of people are saying, well, what's going on there?
Yeah. But on the other hand, do you ever know if Google or Amazon is changing something in the background?
You will never know.
There's no transparency at all.
So going a little bit back to that one is really the point.
These are the building blocks.
And I think what we learned so far is that actually blockchain is built upon five building blocks, more or less.
And Satoshi, whoever it was, mixed them up in a very nice way.
So we have public and private peer production for all of our leads.
We have peer-to-peer network.
We have distributed hash tables.
We have proof of work and we have smart contracts.
All of them are existing in form.
That's coming from agent-based systems.
We are not all talking about AI agents, but agents have already been done 10 years ago or even more than 10 years ago.
So we use smart contracts.
We use the proof.
We use this as the consensus.
We use a lot of hashing.
We use peer-to-peer network to interconnect the different mining nodes.
And use public and private team for the production.
And that brings us now, and I think we go to this one, to the wallets.
Wallets is the access to the Web3.
Who owns a wallet?
Okay, and there's the team.
Maybe we create one.
Okay.
Oh, sorry, Zindal.
A blockchain doesn't have any lock-in.
There is no lock-in to a blockchain.
You can't go to bitcoin.com and revise your password.
That's not possible.
There is no password in a blockchain.
Because a blockchain is only storing in a block addresses.
So address number one is, let's say we use Ethereum.
They have some kind of smart contract.
In a smart contract, you say address number one is owning one token.
Which can be each or whatever.
We do not say this is user number one or whatever.
This is address number one.
And how do we really prove that we are user number one or address number one?
We do that using public and private key encryption.
I think you can cross public and private key encryption.
I just want to remind you what we do here when we encrypt is that we take hello Alice
and we encrypt it with Alice public key.
So this becomes encrypt.
That's the encrypted thing.
So a lot of mixed numbers.
And then Alice wants to read this hello Alice.
It must be encrypted with a private key.
And then she can read it.
So hello Alice.
They're using Alice public key becomes that and then Alice is using a private key and it becomes that.
So that's encryption.
And only Alice can read it because only Alice has the private key.
A public key is well known.
She puts it on her website.
She sends it to you by email.
And you never come from the public key to the private key.
It's not a hash.
But it's similar thing.
You can't go this way.
Okay.
So signing means that hello Bob signs a message with the Alice is signing the message hello Bob.
No, with a private key is using a signature and she's using a private key for that.
Only she knows the private key.
So hello Bob is extended by her signature.
And when Bob receives that information, which is in plain text, it's really in plain text.
Hello Bob.
She's not encrypting it.
She's just signing it.
It verifies that this has been signed by Alice private key by using Alice public key.
So she's signing that for example with a private key.
He's just decrypting it or decrypting this as public key and then it says again hello Bob.
But that's just an affection to them.
And that's the way how a blockchain somehow authenticates you.
When you want to access this particular token here, then you must prove that you have the private key that matches that public key.
Just side step.
It's no longer read the public key is the hash of the public key, but it doesn't matter.
That's assumed with just the public key that is here.
So this is your public key.
And for your pub key, all the transactions are being stored in the blockchain.
So whether that you transmitted a Bitcoin, whether you own a token on the Ethereum blockchain or any other kind of blockchain Zolana or whatever.
It's the public key.
And now you want to transmit that you want to get access to that one.
The minor that you access requires you to authenticate yourself to prove that this is your public key.
Or that you have the private key that matches this public key.
And you can do that for example just by sending you a message, a challenge.
Can send you a challenge.
Five plus five.
Encrypts it with your public key that only you can decrypt it to five plus five.
You send it back the result 10 sign it with your private key.
And then the server can use the public key to check the signature and check the result.
And when it's coming back with 10, it knows that it was you who encrypted it.
So it's a challenge.
So the server is sending you a challenge based on your public key and only you can answer the challenge.
It's asking you a question only you can answer because of your public key.
The woman is having a signature result on the bottom because already the challenge is having a critical layout.
Good one.
So you can just do just an example.
So, but that's the point.
And that's the reason why when you lose your private key.
From the public key will never be able to determine the private key.
When you lose your private key, your money and everything is gone.
Because you will never be able to authenticate or identify yourself as the one who owns the public key that matches this private key.
Therefore, you need to store the private key in a very secure way.
At Toronto, so we have this also.
I mean, this is my smart card.
And on this smart card, I have a private and public key.
And whenever I do a signature, for example, when you start registering for your bachelor or your master thesis, I sign it.
And I do this PDF Adobe.
And then I use the signing tool.
And it asks me for my password.
And then it uses then can access with my password.
Can access the private key and start on your text, the private key and it uses it to sign.
Yeah, or this email.
I mean, having a private key encryption sport has been done.
So that's why I can just give it to you.
I mean, it makes sense.
You don't have the password to open it.
But that's again, the insecure way, although the private key is very important.
If I use a very, very simple password, you can guess it.
So again, the human in the room is always the problem.
So good.
So what I do now is what we do now is that we use wallets and the wallet is just the storage of your keys.
Nothing else.
Very often people say I have five big coins in my wallet.
So there are five bitcoins on the blockchain and in the wallet are the keys to authenticate yourself as the owner of these five bitcoins.
You don't have any NFTs.
You don't have any tokens.
You don't have any theory of your wallet.
You just have private and public keys there.
Nothing else.
So that makes it very valuable.
No.
So therefore you need to make sure that you don't lose your wallet and that nobody else can access it.
So therefore there are tools like MetaMask or even hardware like this kind of memory.
So it's just like a memory stick.
But it's a USB stick that you plug into your computer and this particular stick that's also probably in a private key.
And what you can also do, what you do, we do it next time.
Next time we start with creating a wallet.
So we just create a wallet here and then we do some transactions so that you know how it works.
Maybe next time you can bring your computer, you can also do it on your phone if you like.
What you would need to bring then is some electronic device to download here and some kind of paper.
Because you would need to write down passphrase.
And why you would need to do it, we'll explain next time.
So you need to start that.
You can't do it just in your head.
That's not possible.
So what we do is that when we create the wallet, the wallet creates your private and public key.
And this is then associated with the so-called passphrase and these are 12 or even 24 words that in combination can be used to produce your private key.
And since it's difficult to store your private key because it has a lot of numbers, it's difficult to write down.
You just write down these particular 12 words.
So the 12 words are somehow a key to the private key.
Plus a password to open the whole wallet.
And I think after that you think why is anyone using this kind of crypto?
That sounds so complicated and it is.
That's still the big hurdle to use any kind of blockchain.
It's really the wallet and using the wallet and keeping track of my password and things like that.
Because when you lose it, there's nobody you can ask.
And there are some other wallets.
So for example, if you go to Bitpanda, it's an Austrian provider to buy Bitcoins or any kind of crypto.
It's an Austrian one.
You start an account there.
You do some kind of KYC.
So you need to take this kind of photo check and so on.
And then you need to...
And then you are a customer there.
And once you became a customer, then you can buy crypto.
So you upload your account there with 50 euros.
And then you can say, now I would like to change this 50 euros to 50 Bitcoins or Matic or Polygons or Solana or whatever company it is.
That's possible.
And you only have to provide it to user in your password.
How does that match my argument that you need a copy and try the keys?
They store a copy and try the keys for you.
But they do this.
They create a wallet for you.
They keep the keys and then you lock in.
Then they authenticate you to the wallet and then they do the rest.
And that's why they call this the custodial wallet.
They manage the wallet for you.
Which means that if there is some evil guy or evil lady, she just acts your account and gets your money.
It also means that if you lose your user ID and password, I could lock in and then I do transaction.
And that's how a lot of Bitcoins and crypto has been stolen.
Not because the blockchain is insecure.
Because people hacked into these kind of platforms that do the rest.
Or even there were some people who set up a big platform and they had so much transactions, they just took the money and went.
Let's see, this way.
Build up your own new crypto platform in Aachen.
The new Aachen crypto platform. Here I am.
Barfield license, I sell you Bitcoins.
Do it for two days. There will be enough stupid people who pay you money and then you take the money out.
Probably even before barfield is after you.
Because normally it's regulated by the regulation office's barfield and you can't do it.
As soon as you start it, you get sued.
But that's how it's achieved.
Or somebody is hacking their servers.
Because they are storing a lot of public and private keys.
And if we hack Bitpanda, it becomes very rich.
Because there are so many people who do it.
Therefore, the big recommendation is you can use that for example to get your crypto.
But once you get it there, transfer it from there to your wallet.
To your own wallet.
And that's then the non-custodial wallet, metamask or a stake or whatever.
And the slogan that you need to remember is that one.
Not your keys, not your crypto.
If you don't own the keys, it's actually not your crypto.
It's being owned by a platform like Bitpanda.
It's not you who's owning it.
They are just the gatekeepers in there.
But isn't there then a net flow decline in the amount of Bitcoin?
Because some people will lose their keys.
I'm not sure.
Maybe there are some statistics.
But there will be a lot of Bitcoins who are not on the market.
For example, 50 Bitcoins from Satoshi Hakobodo.
There are a lot of Bitcoins who are not longer accessible.
Which means that we reduce the number of Bitcoins.
Bitcoin is the only blockchain following this principle.
If you were here, you are constantly producing the money.
There is no limit.
And all the other Bitcoins that reduce constant new money.
But they also burn money because you have to pay for running smart contracts.
So we somehow also kill money.
Do some kind of defl-
It's not really a deflation.
It's just to keep them out of the market.
So what's the easiest way and then something else that's lasting.
Sometimes you hear that money is being burned.
I burned a Bitcoin.
Nobody will do that.
But you can burn a Bitcoin.
Or you can burn an E-Bah.
Or just a Matic.
Matic is the worst asset.
You can burn a Matic.
How do you burn something?
Send it to another person.
Send it to the address of zero.
The recipient of your Bitcoin is zero.
Which means nobody has the private key that matches the top key zero.
Complete zero.
If someone finds it, then it becomes rich.
Because suddenly he owns all the things that have been burned.
Burning or deleting crypto means you transfer it to an address that nobody can use as a real address.
Sometimes you can also use burn as the address.
The address is very long.
So a lot of numbers.
And you have zero zero zero zero.
And the last form is just burn.
So if you ever send something to an address from which no private key exists, you burn.
Unless somebody is coming along and creating a new wallet.
And this is suddenly his public key.
Which is commonly the items we have in the world.
And in the universe.
There are more and more possibilities to create public and private keys than we have at the moment.
That's the security.
Okay, good.
So again, thanks a lot for the discussions.
We didn't come through the material that I thought.
I thought we do installing a wallet and discuss all the things.
We do it next time.
So next time we create a wallet, we do some transactions.
We look them up on the chain.
We can find them.
And maybe if you like, you can also create your, you can bring your laptop or whatever.
And then you can in the lecture create a wallet.
You can even delete it afterwards.
Just throw it away afterwards.
And then it's gone.
Just taking 20 keys out of the world.
All the keys that are available.
Okay, good.
Then see you next week.
