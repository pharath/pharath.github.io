---
title: "Web3 and Distributed Ledger Technology - Blockchain Basics"
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

# Blockchain Basics

We didn't give a presentation because we were on an external event at a bank in Frankfurt.
So we gave a complete day on blockchain at a bank.
And this is quite a nice reason because in Germany we have the BAFIN.
And the BAFIN is the organization who's regulating banks and any financial business.
And they set up a requirement that all the banks who are somehow dealing with crypto currencies,
they have to educate the people in blockchain and the rest.
So that's a requirement now that banks have to educate the people who are dealing with crypto in general about the basics of blockchain.
And that bank then approached us to the one day seminar on the whole thing.
And it was quite interesting. I mean there have been people who are really working on blockchain transactions,
in particular Bitcoin. So there are really a lot of Bitcoin transactions every day for their customers.
I think it wasn't a bank who is dealing with private people, more a wholesale bank. They don't do much retail.
So they do a lot of Bitcoin transactions and they actually do that.
They don't do any mining or whatever, what we discuss today.
But they are doing this for external customers.
And there were also people from compliance and they had a lot of questions about security and the rest.
So they were really afraid that the bank might get some compliance issues when they do Bitcoin or any other kind of stuff.

## Segue: SSI Lecture

So what we are going to do **today** is really to jump into **blockchain basics**.
Selin, she told me that there were some questions about some basics.
She said, well, you probably should have the blockchain lecture beforehand.
The problem was that that's normally the case. So we just put it up front.
But hopefully we can discuss some of the questions today so that we clarify what was open there.

## Overview

So what we are going to do **today** is really to go into some **basics on blockchain**.
We use the **Bitcoin blockchain** for that.
And then you see how ~~no~~ blockchain is being organised and what are the different blocks,
what is the **proof of work** and how that works.
And I think that will leave us almost to the end of the lecture so that today you get some basis.

## Recap: Intro Lecture

I think this is something we discussed already last time, the Rai stones on Yap and I think we also discussed the whole issue of that.
What happened there, that is being discussed here with the whole issue of money is the need.
The example was that the Germans painted some black paint on the stones and with this they somehow took away the belief in the whole money.

## slide: "Money is belief"

---

Gemini: "the island of stone money friedman In 1932-33, the Bank of France feared that the U.S. would not stick to the gold standard at the traditional price of an ounce of gold"

In "The Island of Stone Money" (part of *Money Mischief*), Milton Friedman parallels Yapese stone currency with modern gold standards, highlighting that both rely on shared belief rather than intrinsic utility. He notes that in 1932–33, **the Bank of France doubted the U.S. commitment to the gold standard, fearing a devaluation of the dollar**.

- **Context**: Friedman argued that, like the Yapese who accepted the ownership of a sunken stone, the world accepted the value of gold stored in vaults.
- **1932-33 Fear**: The Bank of France feared the U.S. would not maintain the traditional price of gold (per ounce), which led them to convert dollar holdings into gold, undermining the international gold standard.
- **Symbolic Value**: This behavior mirrors the Yapese story where marking stones with crosses (ownership) was sufficient for them to function as money, just as marking gold bars in New York for the Bank of France was sufficient for exchange.

Friedman used this to show that "money" is a social construct rather than just physical metal.

---

Gemini: "'drain' of gold by france from the us"

In the mid-1960s, French President Charles de Gaulle initiated a massive "drain" of US gold reserves by aggressively exchanging accumulated US dollar holdings for gold at the fixed rate of an ounce. Distrustful of US monetary dominance and inflation, this policy forced the US to shed significant gold reserves, intensifying the 1968 **APMEX gold pool collapse**.

**Key Aspects of the Gold Drain**:

- **Context**: Under the **post-war Bretton Woods system**, the US dollar was backed by gold, and foreign nations could convert dollars to gold.
- **The Catalyst**: In 1965, de Gaulle declared the dollar’s special role unjustified, sending ships to New York to convert dollar reserves into physical gold.
- **Impact on the US**: The massive, sustained withdrawal of gold by France, along with other nations, forced the US to reduce its gold reserves to their lowest levels since 1938.
- **Consequence**: This drain was a primary factor leading to the **"Nixon Shock" in 1971**, where President Nixon ended the convertibility of the US dollar into gold, effectively ending the Bretton Woods system.
- **Role of Advisors**: French advisor Jacques Rueff was instrumental in promoting this strategy to combat perceived US inflation exportation.

---

And then there was this other issue that the US Bank refused from sticking to the gold standard,
which means that every dollar is being backed up by gold, which led then to the effect that the French people said,
well, maybe they take our particular gold that's being stored in the bank in New York
and we have to secure it, which led then to the effect that they didn't ship it over the ocean from US to France,
but they just put it into a different corner in the bank and this is how they organised the ownership of the gold.
They just put it into a different corner, it was in the safe,
which then led to the idea that with the US,
that they were afraid that somehow the dollar is no longer valuable in the end,
and which led then to the bank crisis.
This is what Milton Friedman said, who was a Nobel Prize winner.
So it's all about the whole belief in these kind of things,
which is also very typical for Bitcoin.
We just believe that there is some value behind it, or we just give it a value with trading it.

## slide: "Early basics of digital money"

But before we jump into Bitcoin, there was a lot of other issues on digital money.
So digital money hasn't been invented with Bitcoin.
There have been a lot of other approaches which are really interesting
because they show already some of the basic elements that we see later on in Bitcoin.
And I think something that will come up is that Bitcoin is actually a very nice combination
of a lot of very interesting technologies that have already been available before
in different systems.

So the first one is really **David Chaum**.
You may have come across David Chaum already because he's a very well-known cryptographer.
And he did a lot, at least during my studies, he was mentioned several times.
And what he developed in 82-83 was **blind signatures for untraceable payments**.
So he developed already a concept where you could do some kind of anonymous digital money transfer
and he actually brought that into a company, **DigiCash**.
He founded that company, so he wrote a paper, 82-83, about this, where he published this idea and this concept.
Then seven years later, he founded a company called DigiCash.
And that was founded in **Amsterdam**.

## slide: "DigiCash - Deutsche Bank as a pilot partner (1996)"

And there were actually two banks who offered that particular kind of digital money.
One was the Mark Twain Bank (in the US) and the Deutsche Bank.
And we can have a look into this.
I found some press release on that on the Wayback machine.
Do you know the Wayback machine?
The Wayback machine is just where you find old web pages.
And you see here, it's the DigiCash, **eCash** issued by Deutsche Bank.
And they said that **"eCash is digital form of cash that works on the internet where paper cash can't"**.

And they wanted to have electronic cash that

- preserves privacy. That was important.
- It should be secure.
- And they also wanted somehow to make it offline available.

And **the whole idea** was actually during this product that it was in the age, it was the 90s,
it was the age that the World Wide Web really became very famous.
So it was the Web 1, Web 2, as we discussed last time.
So this transition from Web 1 to Web 2.
And it was the same point that people said, well, there couldn't be anything for free on the internet.
We **need to pay for things on the internet**.
So we need to have some kind of micro-transactions.
So just some cent to read an article on the usenet, for example.
And that, I think, was also a motivation for them.

They went bankrupt, unfortunately, around...
So they started in 90s, so in 96 they went bankrupt.
The reason was actually that there was some kind of money laundering taking place there.
But also one reason was that it wasn't being adopted.
I mean, there was Deutsche Bank, there was the Mark Twain Bank.
They went in as a pilot partner.
But there was not really a take-up of the whole idea, unfortunately, at this point in time.

Some people later on say that David Chaum and Nick Szabo both that they were somehow the fathers of Bitcoin in the end.
So some people speculate that either one of them is actually the Satoshi Nakamoto, but nobody really knows.

## slide: "DigiCash - Procedure"

But the really interesting thing is that if we look into this one, is to look how this actually works.
And we discuss this with Alice and Bob, or Alice and some kind of trader.
And what happens there is actually the following:

And we can use this one here to provide you with an example of what is actually taking place there.
Just take some paper over here.
That makes it actually easier to demonstrate.

So what we actually see here, is the following.

It starts with Alice, she writes a **serial number on a piece of paper** and **puts it in an envelope**.
That was the whole initiative.
So we are Alice here and we do some kind of serial number that we put on some kind of paper.
So 4711, that's quite famous here in our area.
So 4711.
And what we do with that is that we put this into some kind of envelope.
So I take this envelope here and then I **send that to the bank**.
Okay, that's it.

So we have this envelope here and what the bank now does is that the **bank now signs the envelope**.
And the bank signs the envelope in a way... Let's say we take this one here.
And now I just sign this here with some crosses.
And what is actually happening is that I don't see the serial number at this point in time.
I don't know where it comes from.
But I signed it off and by magic, you just have to do the magic like this now.
The **signature goes through**.
You know this old paper where it prints through to some other paper.
Here it's some kind of digital way to do that.
And in the end, the bank signed this kind of money.
So what Alice produced is some kind of value.
She says, well, this is, let's say 10 euro, the value of 10 euro and by sending the money to the bank together with that, **she gets 10 digital coins or some kind of paper back that has been signed by the bank** through this **blind signature** and she paid the 10 euros to the bank and now she gets back this kind of paper that has been signed by the bank that this is valid 10 euros.

And this kind of **blind signature** is quite interesting.
We are not going into the deep thing, how it really works.
But it's really very interesting because later on, **Alice can just remove the envelope and then she has this 10 euro note**.

So now she has a 10 euro note with a particular serial number, 4711, and with the signature of the bank.

answer to student question: The bank knows that she's sending that with 10 euros and on the other hand, she's also sending 10 euros.
So she's providing 10 euros, so the bank takes 10 euros from a bank account and with that she gives this back.

So now we have 10 euros, digitally signed by the bank and she puts that into her wallet.

And if I want to trade this now ...
So the next thing is now that **Alice now pays with the digital coin**.

So I just give that to you and say, well now give me for 10 euros some kind of food or beer or whatever.
And what you can do now is that **you send that to the bank**.

So you send this back to the bank and then **the bank checks two things**:

- Do I know this serial number?
- Has this serial number already been provided by someone else?

And if they don't know the 4711, they know there is **no double-spending** on that.

Is the signature valid? Is that really my signature? Yes. And only then you get 10 euros.

So the bank is then actually transferring the 10 euros to you and this was just something like a... in German you would say "Schuldschein" or something like that, that has been completely digitally represented and being exchanged between us.

So that was the first approach of doing that.

And the invention by David Chaum was actually this kind of **blind signatures**.
And the blind signatures allowed the bank to sign something and really putting the signature of those on the envelope,
but also on the content without seeing the content.
They didn't know... so the bank was not able to associate 4711 with Alice.
They just signed something and when you then later on send this check to the bank, then they register 4711.

If Alice would give this to two people, to you and to you, the first one who sends that to the bank gets the 10 euros.
The second one, the bank would say, well, this is **double-spending**, I know 4711 already, it's in my database and therefore I can't pay you out and then Alice will be punished (or will face some other consequences).

So this is how this whole DigiCash worked.

student1: So in how does the bank control when Alice is providing the serial number and that the serial number was not already used by accident.
So in the "flow data line" (the figure on slide 6), it says it only checks the signature and the availability.
So there's no checking of the serial number that Alice provided.
So is every blind signature done with a **unique serial number**?

prof: So what they show is that the serial number is **unique**.
We will have the same later on when we discuss wallets.
I mean also wallets.
When you create a wallet for your crypto currencies, you have to create a serial number more or less that must be unique.
But we just have that long numbers that we hope that they are unique.

student2: Yeah, so if serial number is created by the person and not by the bank,
and the bank doesn't do the serial number, how can we make sure that there is no
other person that created a serial number which corresponds to the serial number created by Alice.

prof: It's almost the same thing.
The point is really that we use some kind of random algorithms that guarantee that we don't produce the same serial number twice.
The solution space is that large that there will be no same serial numbers the same.
So the point is actually, as I said, we have the same problem when you create a wallet.
When you create a wallet, the wallet is also actually just somehow creating your random public and private key.
How do we ensure that there is no same public and private key in the world?
For example, if you just create the same public and private key at the same time by accident at the same time,
nevertheless they must be different.
But that is just guaranteed by the algorithm.

student3: So as far as I know, for example, when you create a wallet and the probability that the wallet was created is not zero, but it's infinitely low.

prof: It's infinitely low. Yeah, yeah.
It's also when we discuss hashes later on.
There is a chance that two people create the same serial number, but this is so small that we just assume it doesn't happen.

student4: **Is the signature of the bank unique to the envelope**, because how do they know, like, okay, Alice gave the envelope,
the person got the 10 euros from Alice, they're going to cash in, **how does the bank know it's 10 euros?**

prof: That was written on that one here.
And Alice is also telling, I mean, this is something, I think it's an interesting point, but I think this is something that the bank could see.
The bank could see, I think actually the bank provides a signature that includes also the 10 euros.
So it's part of the signature itself.
That's what I guess.

student4: So the **signature isn't unique to that envelope?**

prof: No, it's unique to the 10 euros and from the bank.
Yeah, so the 10 euro is not just the signature, but it's also a data that's being encrypted with the signature.

student5: No, I was just going to say, like a Bitcoin, what he was saying, a Bitcoin private key, a 32 byte private key, has the same number of combinations as about the same order as the number of atoms in the universe.
So being equivalent to there being one atom in the universe that is your private key and someone else picking out that exact same atom.
So, yeah, incredibly small.

prof: Yeah, it's incredibly small just by the large numbers.
Yeah, by the large (serial) numbers that you use.

student6: I was wondering **if I have this 10 euro, I cannot pay somebody five euros with it, right?**

prof: Yeah, good question.
How they solve that was also a question I had and I looked it up.
And what they do is when you pay 10 euros, when you say I would like to have digi cash for 10 euros, you get five euros, two euros, one euro and something like that.
Yeah, so they're automatically provided in small pieces and then you use that.
But actually, if you just have one euro left and you want to buy 50 cents, you're lost.
Yeah, that's the case.

We will see later on how Bitcoin solves that.
Yeah, so that's something I think we will be able to discuss today.

So that was DigiCash and the whole idea of DigiCash was this kind of **blind signatures**.
Because the point was an **anonymity**.
They didn't really want to **trace** what people are using the money for.

Actually, there is an approach that I just recently came across, which is called **GNU Token**.
You probably know GNU, GNU.
GNU is famous for open source software and a lot of other developments.
And they are now also working on a solution that they call GNU tokens and they use the same principles.
They use **blind signatures**.
I just came across that because there was a report about restaurants and pubs in Cologne.
And they were asking them about accepting e-payment with a credit card or Apple payment or whatever.
And there was several who said, yeah, we use e-payments.
Some said no, only cash.
And one person said, well, I use cash and GNU token.
So that was the only pub in Cologne actually who was using GNU token.
And then I just came across that and then I saw that they also use these kind of blind signatures.

So that was DigiCash. It was actually the first one.

student: It's not necessarily a question.
It's more of a ..., I googled it, it's called **GNU Taler**.

prof: GNU Taler, GNU Taler, yeah.
Yeah, GNU Taler, not token.
Probably they just want to get separated from blockchain.
Because it's not blockchain based.
That's actually what they say.

## slide: "e-gold Process"

Okay, so the other thing was **e-gold**.
That was something, and that was at the end really killed by money laundering.
And what they did is that users transfer fiat, normal money, yeah, euros or dollars or whatever, to e-gold
or they just provided e-gold.
Yeah.
And then they topped up an internal balance.
So you say, well, I want to have 100 euros in e-gold.
What they do is, they buy some gold with the value of 100 euros,
they put it in a safe and then you get e-gold, digital e-gold in your wallet for 100 euros.
And then when you do some kind of internal transfer, yeah, so if Alice does something to Bob,
then the e-gold reduces Alice's balance and increases Bob's balance and that's an immediate credit effect.
So they just do this internally, I mean you can do this in a relational database that we use.
That's not a problem at all.
And in the end, if Bob then has 100 e-gold,
and he wants to get it into dollars, then they pay it in 100 dollars,
and then they remove the gold or put it somewhere away because then they have left, let's say,
only 900 e-gold left and they put 100 e-gold or gold in the value of a 100 e-gold and they sell it.

So it was actually something backing this to the gold standard.
What we learned at the beginning, where the idea was every dollar is backed up with the appropriate amount of gold.
And here it was just done in a similar way, but they also had this kind of e-gold and that was used because it was sometimes
misused with money laundering, it was all closed down, so they shut it all down and there was no longer a follow-up.
And it was a very centralized process. You had to trust them, you had to completely trust them.
That has nothing to do with decentralization.

## slide: Hashcash

But now we come to something that is really interesting because this is actually a foundation of the proof of work in blockchain
and that was called **Hashcash**. You can actually say that the most prominent feature of blockchain originates in **spam email**.
Yeah, it's so strange, but that's the funny thing to remember. The **motivation** was the **reduction of spam emails**.
So when I started working in our institute, we were working on email actually.
We had the first email system in Europe running, distributed email system running in Europe
and so we were very early email users and at the beginning you looked up your email client once a day and you were disappointed that there were no emails
because there were not many people sending you emails. And there were still discussions at this point in time "will every person have an email address?",
no, probably not. Maybe one for a company. You all know what's happening and that we are just flooded with email. That's the big problem.

And the idea of Hashcash was to say, well, the senders must prove that they have invested computing power before sending the email.
So the border to send email or the hurdle to send email must be very high,
because normally when you send postal advertisements, you have to pay the post office. You have to put a stamp on it. It costs you money.
Sending around a thousand emails costs you nothing, unless in the end you are blacklisted, but it costs you nothing.
And the idea was we have to provide some proof that people invested something. And that was done in the following way:

So the sender had to solve a puzzle before sending the email and to send the result of the puzzle as a stamp to the email.
And the puzzle was actually in the following. They had the following format. That was the actual stamp. And the stamp had this format:

- There was `1` that was the version number. Hashcash version one. They didn't actually come to version two. But Hashcash version one.
- Then the difficulty, how difficult was the puzzle to solve? That's the `20`.
- Then the date `251026`, So 26th of October, 2025.
- Then the sender, `alice@example.com`.
- Then a random number, which is this `30f64fbb`, which is actually just a hash of this `1:20:251026:alice@example.com`.
  - So this `30f64fbb` here is just being created by that `1:20:251026:alice@example.com`. So this changes constantly because internally they don't use just the date, but they also use time in milliseconds. So hashing that `1:20:251026:alice@example.com` results into that `30f64fbb`.
    - phth: ChatGPT:
      - **The random value** could technically be **derived from earlier header fields**.
      - But doing so would reduce security because tokens could become predictable or reusable.
      - Therefore, in Hashcash the rand field is normally generated using independent randomness.
-  And then they said, okay, now this `1:20:251026:alice@example.com::30f64fbb:` is something we can compute easily. But now we have to find a solution. We have to find a number, that in combination with this `1:20:251026:alice@example.com::30f64fbb:` provides us another number that has at least 20 zeros at the beginning. And they did this with hashing. So what they did is they hashed this `1:20:251026:alice@example.com::30f64fbb:` in combination with that `378923`. And the resulting hash should have at least 20 (that was the difficulty) zeros at the beginning.

student1: There is the same effort involved in sending the thing to one recipient or to 100 recipients, right?

prof: Yeah, in some sense. But since you get every email, you send every email in another millisecond, the time will change.
And therefore the random number changes. So in order to get there, we need to have a look at hashes.

student2: Sorry, maybe a security question. But how do you ensure when hashing the version and the sender with the random number
and then again with the solution that you have twenty zeros in front?

prof: I have a demo for that.

Just about hashes. You're all aware of how a hash works. So if not just go here to this `hashgenerator.de` and then you provide some kind of `ABC`
and then you hash it down here and then you see the result.
And the feature of this kind of hashes is that when we just change a little bit of that, the whole result changes.
So we would just say here `ABC`. You see that the hash of `ABC` is this, eg. uppercase or lowercase etc.
That's the whole thing of hashes. And now what I can do now is that I just say, okay, let's say that's my stamp.
So it's one. We need, let's say at least two zeros at the beginning.
And today is the `251113`.
And we use `alice@example.com`.
And what we now also do is we hash that and we put it behind that, let's avoid that at the moment.
We now need just to find a number that provides us a hash down there, which starts with a zero.
And now I can just type in. And now we have it.
Five ones provides us with the solution that we have one zero down here.
So that was easily done.

Yeah, but as I said, we have a demo here, it is a simple proof of work demo.
So we have `alice` here, we have the email address and we have the difficulty in bits.
And now I say, okay, try to find it and you found it.
You now have the stamp.
So here we have this version number one, we have `13` as a difficulty.
That's the date of today. That's the email address.
That's the random number, which is just a hash of that and a shortened cash.
So they just use that here in a way that changes constantly when we produce a new one.
And we used 27,361 tries to find the number such that at the beginning we have 13 bits.
So now here you see just three zeros. This is byte and this corresponds then to this.
So if you play around with that now and we just increase that.
And we say, okay, let's go up once. Let's go to 20.
And now we create the Hashcash stamp.
Now we used already over 500,000 tries.
Yeah, let's increase it further on.
Now it becomes even more difficult.
So we need a lot of tries to find the number that is the combination, eight seconds,
Yeah, and now we have 5 million.
So you did 5 million tries.
So the number of 5 million, `67,000` is then the number such that you get 23 zeros, which corresponds here to 5 bytes.

And this now the original way just to do some kind of proof of work because what you do now is when you receive an email, when you get an email, you do the following:
You just build the hash of that in order to see if you have the appropriate numbers and you can just try this.
So now you get an email and the email contains that.
Actually I haven't tried it, but it should work.
And you just put it in here.
And, yeah, you have the zeros at the beginning.

You obviously would not do this manually, but just your email program would just check
did the sender provide us with a hash cash stamp and you check the hash cash stamp and those emails that actually have a hash cash stamp
those are the ones that have a higher priority because people paid for it with some effort and everything that doesn't have the Hashcash stamp is just being thrown away.
So you don't look into that.

And actually that is something that is the basis for the proof of work of the Bitcoin blockchain.

So that's a really cool thing that we can do.
So let's move this away. We will use this later on for another example.
So that's the very simple solution for that.

student: Yeah, so we don't send our own SHA-1 digest in this case?

prof: Yeah.

student: Okay.

prof: yeah, you don't need to send it because the only thing what you send is just that `1:20:251026:alice@example.com::30f64fbb:378923`.
Yeah, I send this to you and then you just throw it into a SHA-1.
And it is old because at this time they only had SHA-1, they did not have SHA-256 or SHA-512.
So you send this and then I check it and if it is okay, I trust you.

## slide: "What do you think of when you hear the term blockchain?"

The point here is that when you normally talk to people about blockchain, you very often learn about these different issues here, eg.

- people say, well, it's **dark net**. Just recently they got, two days ago, I think in Dortmund, they just caught someone who used also crypto to collect money for paying assassins for murdering politicians or so. So there is still this case.
- It's **energy consumption**. That's also something that you hear very often. People talk about energy consumption whenever they talk about Bitcoin or blockchain.
- Big trouble with the **GDPR** (Datenschutzgrundverordnung).
- Or they talk about "Schlüssel" or **keys and hashes**
- or **banks** and **notaries**.

So that was actually something that we also learned over the past years that when people approached us about blockchain solutions,
that there was a lot heard about this kind of energy consumption and the whole problems or dark net and things like that.
So it always had some kind of **negative connotation**.
This has changed recently.
So there is some kind of change in how people understand blockchain or Bitcoin in general.
And there is more openness now recently.
Obviously in other countries, there's even more openness than in Germany.
And this is probably because I think that at the beginning, the whole blockchain and Bitcoin discussion was very much overhyped, completely overhyped.
So there have been conferences where people said we would get rid of all notaries, you get rid of all banks and all the rest.
And you shouldn't tell that to Germans.
I mean, Germans like banks and notaries and things like that.
On the other hand, there are a lot of countries, obviously, where you probably don't trust the state that much.
And therefore, blockchains can have a big role in them.

## slide: Digitalization (of a Bookkeeping Department)

One thing now is really to understand how digitization in the recent years worked and how people now become more open to blockchain based solutions.
And I often use this kind of picture here to jump into this overall topic.
So that's a picture of an old bookkeeping office around the turn of the century, the 1900s century.
So what you see here is a lot of documents.
You see some people working on the documents.
They do some sort of sorting.
And if you go for digitization of that, how do you digitize that?
What's your first... ?

student: database?

prof: This becomes a **database**.
This becomes MySQL or my MariaDB or whatever you use or CouchDB.
And this becomes your application.
So this becomes your application on top of the database.
And this is how you digitize a lot of documents.
And that's straightforward and that works.

## slide: Network (of Bookkeeping Departments) and the Cloud

On the other hand, you have the problem that there are a lot of companies around and they all have to somehow **interact**.
So we see they all, I mean, this is probably not bookkeeping, some kind of bookkeeping stuff.
But they all have to exchange information.
So what we need are some kind of **interfaces**.
We need some kind of **standards** to do that.
And the big trouble is that very often they thought that they have sent some kind of information to them and they forwarded it here.
But what they received is completely different to what they actually sent.
So the big trouble is that very often we don't have any global understanding or any global consensus or any global status of what we actually did.
And that's the problem.
So very often we have some kind of digitization processes where people need to negotiate what's actually the status of our shared process.
Do we all have the information that are sent to 10 containers or 9 containers?
Some kind of standardization problem in the exchange.
And then there is now the big solution to that, which is in the first step, not blockchain, but the **cloud**.
So now the cloud comes along and they say, okay, come on, give me all your data, put it into the cloud.
And I solve all the problems with the interoperability.
So if we all use SAP or we all use Oracle or a Dexfair or whatever.
We don't have any trouble because we all interact within the cloud and it's their problem to do that.
On the other hand, then we have all the trouble with this kind of locking in to this particular cloud if you just once narrow yourself with SAP or whatever you never get out.

## slide: "We rely on central services and platforms"

And that then gives the whole direction, can we get rid of all these kind of **intermediaries** that provide us the security for transactions?
So again, we have Alice and Bob.
And the whole thing is that at the moment, whenever we do some kind of bank transactions or whatever, we completely rely on central services and platforms.
And these central services and platforms, I mean, they provide us the trust and the security by their organizational processes.
So they tell us that they are doing everything right or wrong or whatever, but actually we completely rely on their process.
If here the Sparkasse in Aachen, if they want to change your bank account, they can just do it.
They have a super user password of the database. They wouldn't do it. They would never do it, but they can do it.

## slide: "How can you manage an account balance between several partners without a central trusted authority?"

And that was the idea then to say, well, can we actually do the following?
That we manage an account balance between several partners **without a central authority**.
That's the whole magic or the whole question that is behind the blockchain.
How can we do that in a decentralized way such that we don't need to trust the intermediary?
And that was at the beginning, when blockchain came up, always the question.
Can we **substitute an intermediary**? Or do we have an intermediary that we don't trust that we want to replace?
Or do we have an intermediary that is too expensive such that we can replace it with a blockchain?

## slides: "tally stick" (2 slides)

And I'm not sure if I showed this example already, did I show that at the first lecture?
student: Yes.
prof: Yeah, then you know about this kind of telestick that actually provided that already in the Middle Ages.
So what we do now is that we develop a digital statistic.
And the digital statistic is now at the beginning that we say, okay, we remove the big central agent in the middle and replace it with a network.
That's the first approach.
And that's really the idea to say the network replaces the overall platform.
The second thing is if we have a network, what we need to do is now we need to also distribute our data.
So we distribute also the data across the whole network.
So this pretty much looks like this into database.
Now I'm not sure if you're using the database next to already, but there you also want to distribute the database, the federated database and things like that.
The point is now that we don't really use some kind of relational database or couching the or something like that here at least different nodes, which are all the bigger nodes,
but that we use a different kind of data structure to do that.
And this data structure is a list of blocks instead of just a table that we have in relational database.
So we substitute actually the standard databases that you all know, like SQL database or something like that, and we substitute it by this kind of linked tables.
And we do that for a very particular reason.
The reason for that is the following.
What we want to achieve is actually to pack transactions that we receive into smaller formats.
And that's put in the next slide and then we go into the different blocks.
The idea is actually or the need for that is actually that we send transactions now into this network and each bunch actually something like a is transferring to be 10 tokens or 10 euros, bitcoins or whatever you want.
And this information is then being received by the different nodes.
But it's not received by everybody because we have some kind of network disconnections.
Some nodes are probably down for some time.
This whole is receiving very early.
This is not because that's a slow line between the nodes.
And the other thing is that this can be further that we have another transaction like the transfer into the 20.
It is only received by this node and by that node.
So we have some kind of problems that we need to solve here.
You have the problem.
How do we achieve in this decentralized approach where every node is completely independent, but just exchanging information about possible transactions.
Among each other, how can we actually achieve in the end a status that is shared among all the different nodes.
If everybody would just put in his database the information that they get after five minutes, we would have already the complete inconsistent state.
Here, if you ask that node what's the account of B, they would say 10 and the account of B is 20.
And if you ask that node, they would say well B and G, they don't have any money.
We haven't seen anything about it.
So we need to find a way that we find some consensus about the transactions that we have received and that we all have the same state.
And that's the magic what blockchain solves, Bitcoin and also the other blockchain actually solves.
So the first thing is that we have to find consensus about it.
And the second thing is also that we have to make sure that the data that is being stored here at the different nodes is not being manipulated.
And if we go back to that slide, let's say here at this particular state, we all know that X transferred to Y100.
And that's something that is being stored here in these nodes in this data.
You have to make sure that that cannot be manipulated, that any manipulation, for example, by just removing 1, 0 from that would be immediately detected.
And that is something we need to solve.
Now, how do we do that?
Let's have a look first on how we actually work on this different data structure.
So what we have is, hopefully you can see a little bit.
Not very good at all.
That's the only one I can find here.
And as you can imagine now we have these kind of blocks here.
So the first thing is that these blocks, let's say that's block number 100, that's 101, that's 102.
And they have been generated.
That's our block chain.
No, that's where the name comes from.
The chain of blocks.
The first thing is that we need to link the blocks among each other.
How do we do that?
You know this already.
How do we link block 101 to block 100?
You say it's in 100 block.
Yeah, I can say it.
So your suggestion is we say, okay, we put a link.
And we say we are linked to block number 100.
Any other guess?
Do we really start the number of the previous block or something different?
You put the hash and the hash.
So we use the hash.
So what we start is actually the hash of block number 100.
Why do we do that?
It means that if we just start 100 and we have the transaction fix,
why 100?
This would mean, okay, we just link to that particular block.
But this link does not provide any information about the content of that block.
If we start the hash of this one, then we connect actually this block to the content of this block.
This is probably what we meant.
And by doing that, we automatically detect, let's say the hash of this one is now,
let's say, two, very simple, again, block of 7-11.
And immediately, if we remove this zero, what you have learned is that the hash changes immediately.
So the hash of that block is no longer for the 7-11, but different, and then the link breaks down.
So just by checking this, we can check this.
And here is another thing we're really to easily exemplify that.
The locked in mining demo, yeah.
So I just have this small demo here.
So we see here we have the block zero.
That's our Genesis block.
We currently have the visibility of three.
And what we can do now, I think, now I have to shorten it a bit.
Okay, so we just now get another block.
And you see the transactions that are being added are always random.
And what we do now is that we link this block here to the previous one in the way that the previous hash is this one,
C02, which corresponds to that particular hash.
And then I get another block.
I just link this to 089 to this particular one.
So each block actually contains the link to the previous one, but not a number, but as a hash.
And as soon as I change something, and I want to manipulate this here, let's say this is 100, you can see.
So immediately we see that the data has been manipulated because the hash and the non-satve come to that,
which is similar to the proof of working hash cache, they don't meet each other anymore.
They are wrong.
And also here we see that the link is this one.
So you see the previous hash, what we believe is 0089 and the hash is now seen to 33.
And by that you immediately detect that something manipulated the data in our blockchain.
And that now makes it possible that these kind of blocks or that these nodes can run everywhere.
So we don't need to trust any more the server itself, because the server delivers us some kind of data,
and we can immediately check if the data is correct.
So that solves the problem of manipulating the data that is being stored.
But now you could argue, but I just could recompute the hash when this node becomes 4712,
that I can just control the 712 here.
And then I just manipulate all the different links within the block to do that.
So there must be some kind of difficulty to do that.
It must not be easy to do that.
And this brings us now to the point, if we have all the different transactions in our network,
so one thing is that it must be difficult to manipulate the data structures here.
It shouldn't be easy, that's one thing.
The second thing that we have to solve is actually who of these 7 nodes is the one who shall add a new block to our blockchain.
So the problem is the following. We have now our blockchain here.
And let's say here, just a blockchain of two blocks.
And now we get a lot of new transactions.
We say A to B, 10 and D to C, 5 and A to D, 5.
So these are new transactions that we have to proceed.
And now we would like to add this as information to our blockchain.
And everybody wants to do that now. Everybody wants to each node here,
wants now to add this information to the particular blockchain.
And how we do that is by now creating a new block.
So we create a new block.
We put the three transactions, one, two, three into the block.
And now we would like to link that one to this here.
So first we take the hash of that one, so we take the hash of 101.
We take a timestamp, take the time, let me write into our header.
And we need to do something in addition.
Because we need to find a solution that only one is being selected,
almost one is being selected as the one who is in this network
who can add the block to our blockchain.
So the problem we have to solve is who is the one who is adding this block to the blockchain.
So we have the question that we are now the network, so we have about 30 nodes.
And everybody of you is now trying to add this block to the blockchain.
How shall we select who the miner or who the node is?
The person who finds a valid nonce in order to produce a low enough double-shot-reflexible solution.
That's already the perfect answer. Why should we use the oldest one?
The oldest what? The oldest miner. Let's say we are all personal.
So we could all check who is the owner of this particular computer and we use the oldest one.
Why should we do that?
Concentralize.
Why shouldn't we do some kind of round robin?
So which means that first it's even, then me, then you, then you, then you, then you.
Could also be done because then it's not always the same. We don't have centralization.
We just use...
Either it's not dynamic, like we need to fix the amount of people and we all know about the participants
or it's like a mess to that everybody knows who is in charge.
The problem is that then it becomes predictable.
It becomes very much predictable who is going to be next.
What we want to have is that you don't know who is going to be the next.
Such that you can try to manipulate because then you foresee what's coming up.
You try to somehow manipulate it.
You want to have it completely random.
Okay, so another suggestion.
If you do it randomly, I could just think about a random number
and then between one and a hundred and the person who is being closest to that gets it.
Why shouldn't we do that?
You yourself are being closest if you want to.
Let's say I'm not providing a minor, but the problem is again we are centralizing.
Again, you have to trust me.
I become the central trust point and I do it correctly.
So these are all approaches that are somehow not applicable
if you want to have a complete random solution
and if you want to have a non-predictable solution
and if you want to have some kind of trusted solution.
Therefore, we do what you suggested.
We use the proof of work that we learned from hash catch.
You may not cook a work or pay because if you pay to compute this one or two times
it's the same as you may pay, but not for this.
I'm like one manipulation is already enough to manipulate the network.
Therefore, it must be very cost intensive to do it.
Not in the way that I did it here was just a few, let's say one million tries.
So we need to find something where we have really hard work to do that.
And it's the same principle.
But isn't it very inefficient if you want to use the conformity of that translation?
Yes, it is.
Yes, it is.
It's inefficient if you compare it to the amount of energy that you need.
And therefore, but that's the next lecture, you think about alternative solutions.
But the point is really first to understand how Bitcoin actually works and how this works.
If you were to use the hash catch, when we're going after probability theory, right?
Let's say in Georgia I would need one million tries per average.
Then wouldn't that be the risk of several parties finding the solution at roughly the same time
and then by connections, then yes, you'll get the whole thing?
So let's put this on, we discussed that.
Okay.
Yeah, good thing.
But we're actually doing now with hash catch, what we say is, okay, now you need to find the nonce,
the number only used once, that was the number we were computing with hash catch.
You need to find the number only used once, that depends on the content of the block.
So we take our transactions, we take the time, we take the hash of the previous one
and adding all that, so we have the hash of the content plus nonce.
And it will hash that.
This must be below a certain number, which means it must have at the beginning the number of zeros.
That's how we interpret below.
And that's our requirement.
And the minor, the note or the minor, they're often called minor because you see later on why they're also called minors.
The note who is doing that first is the one who is sending this information that we found in the new block
to the network and then the network checks it.
So actually he's putting a stamp on that, like hash catch.
And he's putting a stamp that he found a solution to the puzzle and he's sending that to the only blocks.
And let's see how it looks like also here.
What we did here is actually, here we had difficulty 3 and we wanted to have here it's really 3 zeros.
To be able to just reset the blockchain, okay, let's create a new one, okay.
With the difficulty of 3, we need at least 3 zeros that we find here.
And the nonce was 2003 and 57. That was easy to do.
Let's increase it.
Let's say again, we want to have 5 and now we create a new block.
Okay, we now have 5 and now we need 500,000 tries.
So the nonce of 575,000 in combination with this here then provides us 5 zeros.
Now let's do it with 6.
It takes already longer.
Now you see.
10 seconds now to find this.
Okay, and now it will increase it higher.
It takes longer.
So, you see now we are.
And now you see, if everybody who's doing that now,
if all of you would now start this application,
then you would have one who's first.
Who would be the first one at the moment?
The one with the fastest CPU.
The one with the fastest CPU.
And the one with the fastest CPU will win.
And he will probably win every time.
Therefore, there is a race of just becoming the fastest one.
Which means that at the beginning was just a race.
Really, we have machines that do good ashing.
So there is a particular kind of hardware.
And this particular kind of hardware is very good in computing ashing.
So that's actually what you now do.
What other kind of idea?
So the first is everybody of you gets a, gets a Dell laptop with the same CPU time.
What would you try in order to become first?
So we all have the same.
You all have the same software.
We have any other kind of guesses?
It's a different approach than the other.
I would time it at the same time.
Different approach could be, all of them, they start with one.
And you start, so this program is just counting from one upwards.
You probably would say, I start with 10,000.
Because if all the others do, because they start with one,
I start with 10,000 and then they're probably become different to all the rest.
I mean, every miner has a different coin-based transaction to their own address,
which would change the hash and then change all the,
so they would all be searching different hash spaces.
Different hash spaces.
That's something you're right.
What I said assumes that we all have the same, have the same content hash.
But that's not the case.
Because you may have received other transactions than you.
You have received these three transactions,
but you may have received these three transactions,
because something else.
And again, you have your own address, which is also being involved here.
So it's your own address that is in here.
So it's an address that you also add.
And there will also be a very special transaction that also makes it different.
So everybody has actually another content hash for which he needs to find the nonce.
So you could say, well, I'm not starting at one,
because I think starting at one million is probably better,
because then the other hash can be found easier.
I have no idea of that.
But you can do some kind of guessing.
But there are other ways to do it.
Isn't there like the conflict that if we make it too easy,
too many people would find it at once,
and make it too difficult, and no one will ever find it.
And you see here, my computer is now shortly before burning.
It's now trying to find, specifically, seven, it's really already working a lot.
About this, we also discussed that.
I mean, these are all features that are built into Bitcoin,
which are not problems, but actually features.
Okay, so let's...
Computing.
But there is still one way that you could do as a group to become faster.
Like a mining pool, you mine together.
Yeah, you're going together.
So what you could do is that you two say,
okay, we get the same data,
we become one node to the outside,
but internally we have two computers,
and you start at one, and you start at one million to find it.
Which means that you now have two computers trying to find the nodes.
Obviously, you don't start at one, because that would make sense,
but you start at different nodes, and then you use that.
And then you two have almost a double power than all the others,
which means that you will then be almost the first node.
And that's the mining pool.
You just put together all your computing power,
and you distribute the solving of the puzzle to all your computers that you have available.
Okay, maybe I didn't understand a part of it.
So like, we have two computers,
but doesn't it just mean that they're looking at the same time
in different spaces to find a solution?
But not necessarily they're going to find first,
but they can, because it's probability is a bit higher,
but they have two slow computers, one very fast computer can beat them.
Definitely, yeah, that's the case.
So if they have two slow laptops, and you have a very fast one,
then you probably overrun them.
But nevertheless, it may be that most of the time it's the others.
And that makes it so interesting.
But you can't really predict what's going on.
Maybe we'll talk a little bit, but right now I do not understand
why do I want to be the first one to find a good one.
Right, now, from my perspective, everyone gets the same data eventually,
so every data will land on the blockchain.
Yeah, and this question is something that we discuss now.
So it's really the point why.
Let's keep it working.
The point actually now is this is the mining process.
That's the whole mining process.
It's so easy.
It's just a loop that we do, we hash the block header plus the nonce,
and then we hash it again, and then we see is this below the difficulty,
so 4070 was what we discussed here.
No add one to the nonce, and yes, send the block to all neighbors
that you have available, and this is what's happening now,
that you send all the blocks to your neighbors with the solution of the crypto puzzle,
and then what they do is they just check it, and that's easy.
They just need to check is this hash of the content plus the nonce in total
as the certain amount of zeroes.
Fine, then that is being accepted.
So now we come to some questions.
One question is why should I do that?
What's my incentive?
What's my benefit?
You get the block reward.
You get a block reward.
You get money.
But how do you get money?
Where do you get the money from?
In addition to the block reward, you also get the gas and mining fees.
Gas is something we discuss when we talk about material.
But transaction fees.
Transaction fees, yeah.
Transaction fees, yeah, that's what you get.
First thing is that a transaction within these kind of blockchains,
they don't do it just for fun.
They want to earn money.
It's not for free.
A blockchain transaction is not for free.
If I want to send you some coins, they want to send you a token, some crypto or whatever,
I have to add fees like with a bank.
You also pay.
I mean, this is something where very often people think, well,
Bitcoin transactions are for free.
That's what it differentiates from bank transfers.
No, they cost you money.
And very often it's calculated in an amount that depends on the current traffic in the network.
If the Bitcoin network or the blockchain network is busy,
you pay more fees than if it's not busy.
This is Uber.
You all know that you want to go on a Saturday evening out of Aachen, back home.
Uber is expensive and then Monday morning is for the cheese.
So the mining fees really depend on the traffic in the network.
And what these people do here is that when they get all the transactions,
they select those transactions with the highest fees,
which means that when you submit a transaction into this network and you don't provide any fees,
you could do that.
You could say, please, I would like to send you one Bitcoin, but I don't provide any mining fees.
What happens?
This will not be included.
All the miners, they want to have transactions with high fees.
So they probably just put it aside.
And only if there probably is no traffic at all,
they probably put it into their list of transactions.
But maybe never.
But it could be taken a day or a week after the transaction.
But is there for every block just one transaction?
No, every block has...
3,000 to 1,000.
Okay, but it's a fixed amount.
It's a fixed amount.
That's been discussed, always discussed,
because the community also depends on the blockchain.
At the beginning, there was not that many transactions, but they're now increasing it.
Really, you can't go up to it.
So the whole process doesn't make much difference.
Yeah, because otherwise you could just put every transaction in.
Yeah, but they also want to get money.
Yeah, if one might not probably take every transaction limiting of the people that spend money on.
So, first thing is they get transaction fees.
And they get a reward.
They get a block reward.
So what they do is...
And that also makes every content different.
Even if you all get the same 3 transactions, you're okay.
You all have your own ID, which is being stored in your block header.
And there is one particular transaction where you say,
just one, let's say one Bitcoin, BTC, to me.
So you just include one transaction.
Then you put one Bitcoin to your own wallet.
So you just create this kind of Bitcoin out of nothing.
It has never been there.
You just say, my account has now plus one Bitcoin.
And at the beginning, when Bitcoin was launched in 2009, that was 50 Bitcoins.
So for every block, you got 50 Bitcoins, which would nowadays be 50 times 100,000, 5 million.
That was 2009.
So 50 Bitcoin, every block.
That means with every block that we add to the blockchain,
the money that is available is being increased by 50 Bitcoins,
which is quite some inflation.
It's like our Bundesbank or the European Bank, who is constantly 20 Euros.
We print Euro, we print Bitcoins here.
But only every 10 minutes.
The 10 minutes is something that comes into the game.
There is also something.
We don't want to have inflation too fast.
We want to have inflation in a calculated way.
So therefore we say a block should, among all yourself, be found within 10 minutes.
And the difficulty is therefore adapted such that the community of all miners finds a block on average every 10 minutes.
But then in the case of Bitcoin, the block reward gets hard every 210,000 blocks.
That is something I would like to come to.
So let's see what our miner is doing.
He found it.
462 seconds.
And at C difficulty 7, the nonce is 185,808,000.
So it took him 46 seconds, which is about 7 minutes.
Oh, we are quite there.
So for us, the difficulty of 7 would mean that we can be in the game.
But I think the current difficulty is about 14, or 15.
You see how dramatically it increased.
We need to find this knowledge.
So what we want to have is a block every 10 minutes.
And with this particular block, we create new tokens.
We create new Bitcoins out of nothing.
So we increase the number of Bitcoins that are available.
And this should be done almost every 10 minutes.
And if you all now invest in computing power, and suddenly you find the block every 5 minutes,
what do we do?
We increase the difficulty.
And by that, we bring it back to 10 minutes.
I think this is extremely elegant.
You just increase the difficulty back to 10 minutes.
Suddenly, you all say, well, now I'm doing large language fine tuning.
So I hate all this Bitcoin stuff.
My computer now on fine tuning and large language models.
So suddenly, we find blocks only every 20 minutes.
We reduce the difficulty.
So we just adopt our difficulty to the current power that is available,
the current computing power that is available is in the network.
There is certain harm in it.
So not really like minor points and stuff, but like the step of 67 was of time factor, I think, 70.
So what is there is no probability to reduce it from 20 to 10,
but if you reduce it by 1, you already get 5.
So you mean that 1, if you just increase it from 100 and 201,
you don't find it anymore because the solution space is so small.
That's interesting.
Could be, could be that this happens,
but then they probably use some other kind of difficulty measure.
So something they're creating.
It's an average over the last 2016 blocks,
and it can only be adjusted up by 4 and down by a factor of 4.
But depending on the average, it can go up also a very, very small amount,
or down a very small amount.
And you see here, I'm using now seven bytes.
If you do it on bits, then it becomes much higher, right?
With hash cache, I do it on bits, here I do it on bytes.
And a byte is just 8 bits that you need to find.
So it becomes, so the solution space is every number becomes really small.
And you get some time, right? That's probably the right answer.
Okay, so we now, we do it because we get money.
We get bitcoins as a minor every time.
We increase the amount of bitcoins that are in the market.
We have some kind of inflation,
and if you already have too fast computers,
you just increase the difficulty.
Nevertheless, it means every 10 minutes, 50 bitcoins.
That's too much.
At the beginning, we wanted to have a lot of coins immediately in the market.
And we want to have, flood the market with bitcoins so that we get some kind of,
not just a big market.
After some time, this becomes problematic.
You need to reduce it.
What we do is, we just halve the amount of bitcoins that you get for the money,
from 50 to 25, from 25 to 12.5, from 12.5 to 6.75,
and from there to 3.5, something which is the current amount that you get.
And you sometimes get these kind of spam emails,
Bitcoin halving, buy Bitcoin, Bitcoin becomes more valuable.
That's the Bitcoin halving.
They don't halve bitcoins.
They just halve the amount that the miners get,
which means that actually, at the moment, it's 3.75,
and then after a certain, I think, yeah,
3.125.
3.125, ETC with everyone.
And next time, when we do the halving, that is 1.6 something.
And we do it every 210,000 blocks, as I showed you.
So we do it every 210,000 blocks,
which means that if you divide it by 10 minutes,
it's every two years or so.
Every four years.
So every four years, you do the halving.
And the last one was what, one and a half or two years ago.
April 2024.
Yeah, I agree.
You see, I'm not dealing with this event so much.
So that's something that will happen next time.
So the next one will be in...
2028.
2028, yeah, so 2028, still great.
So two years ago, almost.
And then...
So, but if we halve and halve and halve and halve,
after some time, there will be no incentive,
there will be no proper reward.
How will the system then still be running?
Why should people...
What, no, transaction please.
That's the idea.
The idea is that then, and this will take place in year 2000...
2140.
Yeah.
2140.
That's the year when there will be no more block rewards.
So still more than a year ago.
And until then, they just speculate,
or the inventors speculate that then this will just be based on transaction.
I don't know, I just wanted to look this up.
Could be a nice task for you to look this up.
What is the count ratio between block reward and mining fees?
It could already be that we soon reach that.
Miners get more mining fees, more transaction fees than what they want.
Because there are almost 10,000 transactions in there.
And imagine that there is someone sending five bitcoins.
And if you get them, one or two percent as a transaction fee,
you get already quite fast.
This is why people do that.
Actually.
That is why people do that.
This is why people do that.
Actually.
That is why you can now start mining.
You wouldn't have much luck, you see, with this example here.
What's the minor from putting more bitcoins in itself while mining the block?
Yeah.
If you put in 10 bitcoins,
and you send that block to all the other blocks,
to other miners, what will they do?
Reject it.
Reject it and reject it.
You are just playing for us and probably they are just like this.
From then on, you are gone.
You have to find a new IP number then.
Because they are like this.
So that's something that would work.
So I think what we learn now is that we do,
what we learn now is that how this is all being organized.
So we have these kind of blocks.
We learn how you actually, why we do that.
The incentive for doing that, to invest money and computing power into that.
Because you get a rock reward.
Beyond probably just the idea of I would like to be part of the community.
Because I believe in Bitcoin and cryptocurrencies also.
And one question, one thing that you also should know about is this.
And then stop because that's probably the right rate.
And then next time we discuss what happens if we have two blocks.
Being found.
How the actual transaction works.
But this would open up now pretty much.
Merkle tree. Do you know Merkle trees?
I mean it has nothing to do with our old chancellor.
Same name but it's not.
What's named after her.
The Merkle tree is the way we organize the transactions in a block.
So I was actually cheating here.
I said, well we have a header.
And below that we have a list of all the transactions.
This is not what we do.
We do it a little bit of a way.
So what we actually do here is that we have.
This kind of organization of each block.
So each block has the previous hash.
That's what we have.
We have timestamp, also code.
We have the nodes.
And we have the root of the Merkle tree.
And you see we are not listing our transaction just in a list.
And we hash that.
What we do is we take transaction number zero.
So we have four transactions in this example here.
Zero, one, two, three.
And then we take the hash of transaction number zero.
And we have to hash of number one.
We have two hashes.
We combine that into a hash.
And do the same here in our tree.
And then actually we hash these pairs against each other.
And that gives the root.
So actually some miners which are called light nodes.
These light nodes, they don't contain all the information here.
They just contain that.
So they just work in the network.
And they just work in the network.
And they do some kind of proof that the new node is correct or things like that.
But they don't contain all the information because that information is quite large.
Downloading the whole Bitcoin blockchain.
So really the data is gigabytes.
I'm not sure 900 gigabytes or so.
800 gigabytes.
800 gigabytes.
That means that if you want to now become a miner to download the software,
it takes a minute.
Installing it, probably just a minute.
Downloading 900 gigabytes takes days.
So although, but you would then run something like a light node,
which are some of the lighting network or things like that,
that would be run on the rest of the time.
But that's something we will probably discuss later on.
So what we actually organize this is that within the core data,
we just have one hash.
And that's the hash of all the transactions that is combined as this Merkle tree.
And when I saw this first, I thought why?
I mean, why do we hash always the pairs and then we hash the pairs
and put it there.
We could just hash that and put it there.
Do you have any idea why?
Why do we do this kind of tree?
I mean, computer scientists, they love trees, obviously.
Yeah, so we do everything in trees, but the list and then hash it.
I was thinking maybe we can, using that somehow find out if somebody cheated.
Where are you cheated?
No, no.
Any other guess?
To avoid like going over the side of the block, having this one hash.
Yeah, but that's the same.
If we, why don't we just hash this pair-wise and we put the hash in there
or just taking that and put the hash in there.
That's almost the same.
That can work with something too.
Yeah, it goes into the right direction.
If I manipulate H1, the hash to three would still be correct.
I could identify which way I'm going to go.
Yeah, it goes also in the right direction.
Yeah.
It allows also miners to change the Merkle route slightly.
In case, for example, they run out of non-space,
you can also change a little bit the Merkle route and then go over a new hash space.
I'm not sure that's the answer you're looking for.
No, I don't get this.
I'm looking for another solution.
The point is that what happens if you want to check if a certain transaction is actually within this example here.
So we store all the transactions and we just hash everything and put this into our business ticket route.
So we have all the transactions and we hash it and we get it into TXROO.
And we have that one.
If I give you now a transaction, I give you transaction number one, which is this one.
This is TX0 and this is TX1.
I give you transaction number one.
What do you need to know if this transaction is in this particular block?
If I give you also the TXROO.
So the question is the following.
I give you a transaction and I claim that this transaction is actually being in block number 11.
Which kind of information do you need to know to check if transaction number one is in block number 11?
In this example.
In this example first, using the mark to do the reverse hash.
Not the reverse hash but like the bit mask.
This is not concatenated.
They are just hashed and hashed and hashed.
All the transactions.
In this case, I need all the transactions to check if transaction number one is here.
So I need 1000 transactions or in this case, let's say, I need 4 transactions.
Let's say it's just a 4.
I need 4 transactions to check if transaction number one is in this block.
How many other transactions information do I need to know if transaction number one is in this block using a mercury?
I need transaction number 0.
I compute this, I compute that and I need the hash 23.
I don't need this and this so I need 1, 2, 3.
The hash 0 would be already enough right?
The transaction 0 could be anonymous.
The Yama, this is not stored.
The only thing that is actually stored in the node is this.
That's the interesting thing and that also gives these light nodes some kind of sense.
If I claim that my transaction is in block number 12 and this node doesn't know all the other hashes,
I just need to provide my transaction, transaction number 0 and that can be computed if this is the same as this one.
So normally I would need to provide all the false transactions.
Here I just need to provide 3.
Now imagine that you don't have such a simple mercury.
We have a much larger mercury where we have much more nodes.
Then again the information I need to provide to check that a transaction is in the mercury is normally only the number of levels of the tree.
You automatically get up and you just need to provide the pairing hashes and that reduces computing power and data that you need to provide.
How do I know in this case for example hash 23, 24 in the root of the store?
This is then often stored here within the node.
That's how they know that.
They need to do less comparisons.
This is the head-out.
Then the block itself really contains all the data.
For example if one node wants to communicate that to another node,
then it just needs to send this, this, this and that.
Actually in mercurys they have been invented before blockchain.
They have been invented in communication technology.
In communication technology you want to save computing power and you want to save bandwidth.
If you want to communicate that to someone else,
you just need to reduce the amount of information instead of the whole information.
With a reduced amount of information you can check that part of the information is contained in that information.
So in mercurys, maybe someone can look it up on where mercurys came from.
I believe they came from communication technologies.
They wanted to make communication more like that.
Okay, good.
With this we can stop today.
What we do next time is then to discuss what happens if we have more than one front-section.
How do we actually send if we have one bitcoin?
But I only want to pay you a half of bitcoin.
How can I cut a bitcoin into two pieces?
I don't know.
I know that you don't know that.
Next time we discuss that.
Some other very interesting issues.
Then we also go to some other kind of blockchain principles.
Because at the moment bitcoin is the only blockchain that is still using 2K.
But there are others in the group of state.
Then we discuss how the group of state works and what they actually do.
I will think if I can give you already some kind of exercises.
Maybe it will just be some questions that you ask for yourself.
So that you can recap that you understood all the things.
Okay, good.
See you next time.
Next time.
Okay, exam date.
Normally I do it in week one or two after the last lecture.
Would that be okay?
Is there the last lecture is on?
The last lecture is on the 4th of February.
And on the tips.
So in the week of 9 to 13.
Let me just check that we don't conflict with Ben's Carnival.
From the 12th to the 18th.
Okay.
What is this?
This is a little bit...
Ah, okay.
I mean I don't care but at least I would say it's a little bit of carnival but you have to learn.
Would be the 11th.
Okay.
I'm already sure.
Yeah, okay then it's not okay.
The Monday the 9th.
Can we make a full report?
Okay, yeah.
But in general it would be more appropriate for you to do it just shortly after.
And not just weeks after.
Okay.
Because then we could also do on the 4th we could do some kind of Q&A session.
So you can then come up with questions and I can get you some.
There will be no people often ask do I have a test exam or there will be no test exam.
But I can give you some typical questions or we can do some kind of Q&A.
Okay.
Do you have certain exercises today you were posted?
I checked.
I will probably do it on Friday because today I can't do it tomorrow.
I know way.
And then to the next week when?
Next week or next week.
I think the first exercises will really come more when you start and then Ivan is telling you something about smart contracts programming.
At the moment the exercise will be more that you just recap some of the content.
Okay, good.
So then see you next time.
Okay.
Okay.
Okay.
Okay.
Okay.
Okay.
