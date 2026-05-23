---
title: "Web3 and Distributed Ledger Technology - Blockchain Basics"
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

So the first one is really <span style="color:red">**David Chaum**</span>.
You may have come across David Chaum already because he's a very well-known cryptographer.
And he did a lot, at least during my studies, he was mentioned several times.
And what he developed in 82-83 was **blind signatures for untraceable payments**.
So he developed already a concept where you could do some kind of anonymous digital money transfer
and **he actually brought that into a company**, <span style="color:red">**DigiCash**</span>.
He founded that company, so he wrote a paper, 82-83, about this, where he published this idea and this concept.
Then seven years later, he founded a company called DigiCash.
And that was founded in **Amsterdam**.

## slide: "DigiCash - Deutsche Bank as a pilot partner (1996)"

And there were actually **two banks who offered that** particular kind of digital money.
One was the **Mark Twain Bank** (in the US) and the **Deutsche Bank**.
And we can have a look into this.
I found some press release on that on the Wayback machine.
Do you know the Wayback machine?
The Wayback machine is just where you find old web pages.
And you see here, it's the DigiCash, <span style="color:red">**eCash**</span> issued **by Deutsche Bank**.
And they said that **"eCash is a digital form of cash that works on the internet where paper cash can't"**.

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
So just **some "cent" to read an article on the usenet**, for example.
And that, I think, was also a motivation for them.

**They went bankrupt**, unfortunately, around...
So they started in 90s, so in 96 they went bankrupt.
The reason was actually that there was some kind of **money laundering** taking place there.
But also one reason was that **it wasn't being adopted**.
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
She says, well, this is, let's say 10 euro, the value of 10 euro and by sending the money to the bank together with that, **she gets 10 digital coins or some kind of paper back that has been signed by the bank** through this <span style="color:red">**blind signature**</span> and she paid the 10 euros to the bank and now she gets back this kind of paper that has been signed by the bank that this is valid 10 euros.

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

And if they don't know the 4711, they know there is <span style="color:red">**no double-spending**</span> on that.

Is the signature valid? Is that really my signature? Yes. And only then you get 10 euros.

So the bank is then actually transferring the 10 euros to you and this was just something like a... in German you would say <span style="color:red">**"Schuldschein"**</span> or something like that, that has been completely digitally represented and being exchanged between us.

So that was the first approach of doing that.

And the invention by David Chaum was actually this kind of **blind signatures**.
And the blind signatures allowed the bank to sign something and really putting the signature of those on the envelope,
but also on the content without seeing the content.
They didn't know... so the bank was not able to associate 4711 with Alice.
They just signed something and when you then later on send this check to the bank, then they register 4711.

If Alice would give this to two people, to you and to you, the first one who sends that to the bank gets the 10 euros.
The second one, the bank would say, well, this is **double-spending**, I know 4711 already, it's in my database and therefore I can't pay you out and then Alice will be punished (or will face some other consequences).

So this is how this whole DigiCash worked.

---

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

---

So that was DigiCash and the whole idea of DigiCash was this kind of **blind signatures**.
<span style="color:red">Because the point was an **anonymity**. They didn't really want to **trace** what people are using the money for</span>.

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

So that was <span style="color:red">DigiCash. It was actually the first one</span>.

student: It's not necessarily a question.
It's more of a ..., I googled it, it's called **GNU Taler**.

prof: GNU Taler, GNU Taler, yeah.
Yeah, GNU Taler, not token.
Probably they just want to get separated from blockchain.
Because it's not blockchain based.
That's actually what they say.

## slide: "e-gold Process"

Okay, so the other thing was <span style="color:red">**e-gold**</span>.
That was something, and that was at the end really **killed by money laundering**.
And what they did is that users transfer fiat, normal money, yeah, euros or dollars or whatever, to e-gold
or they just provided e-gold.
Yeah.
And then they topped up an **internal balance**.
So you say, well, I want to have 100 euros in e-gold.
What they do is, they **buy some gold** with the value of 100 euros,
they **put it in a safe** and then **you get e-gold**, digital e-gold in your wallet for 100 euros.
And then when you do some kind of internal transfer, yeah, so if Alice does something to Bob,
then the e-gold reduces Alice's balance and increases Bob's balance and that's an immediate credit effect.
So they just do this internally, I mean you can do this in a **relational database** that we use.
That's not a problem at all.
And in the end, if Bob then has 100 e-gold,
and he wants to get it into dollars, then they pay it in 100 dollars,
and then they remove the gold or put it somewhere away because then they have left, let's say,
only 900 e-gold left and they put 100 e-gold or gold in the value of a 100 e-gold and they sell it.

So it was actually something backing this to the **gold standard**.
What we learned at the beginning, where the idea was every dollar is backed up with the appropriate amount of gold.
And here it was just done in a similar way, but they also had this kind of e-gold and that was used because it was sometimes
misused with money laundering, it was all closed down, so they shut it all down and there was no longer a follow-up.
And it was a **very centralized process**. You had to trust them, **you had to completely trust them**.
That **has nothing to do with decentralization**.

## slide: Hashcash

But now we come to something that is really interesting because this is actually a <span style="color:red">**foundation of the proof of work**</span> in blockchain
and that was called <span style="color:red">**Hashcash**</span>. You can actually say that the most prominent feature of blockchain originates in **spam email**.
Yeah, it's so strange, but that's the funny thing to remember. The **motivation** was the **reduction of spam emails**.
So when I started working in our institute, we were working on email actually.
We had the first email system in Europe running, distributed email system running in Europe
and so we were very early email users and at the beginning you looked up your email client once a day and you were disappointed that there were no emails
because there were not many people sending you emails. And there were still discussions at this point in time "will every person have an email address?",
no, probably not. Maybe one for a company. You all know what's happening and that we are just flooded with email. That's the big problem.

And the idea of Hashcash was to say, well, the **senders must prove that they have invested computing power** before sending the email.
So the border to send email or the hurdle to send email must be very high,
because normally when you send postal advertisements, you have to pay the post office. You have to put a stamp on it. It costs you money.
Sending around a thousand emails costs you nothing, unless in the end you are blacklisted, but it costs you nothing.
And the idea was we have to provide some proof that people invested something. And that was done in the following way:

So the **sender had to solve a puzzle** before sending the email and to **send the result** of the puzzle **as a stamp** to the email.
And the puzzle was actually in the following. They had the following format. That was the actual stamp. And the **stamp had this format**:

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

prof: yeah, you don't need to send it because <span style="color:red">the only thing what you send is just that</span> `1:20:251026:alice@example.com::30f64fbb:378923`.
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

prof: Yeah, then you know about this tally stick that actually provided that already in the Middle Ages.
So what we do now is that we develop a **digital tally stick**.

## slide: "Network replaces platform"

And this tally stick is now at the beginning that we say, okay, we remove the big central agent in the middle and we replace it with a **network**.
That's the first approach.
And that's really the idea to say the network replaces the overall platform.

## slide: "Database is distributed"

The second thing is if we have a network, what we need to do is now we need to also **distribute our data**.
So we distribute also the data across the whole network.
So this pretty much looks like a distributed database.
Now I'm not sure if you have visited the database lecture already, but there you also talk about distributed databases, federated databases and things like that.
The point is now that we don't really use some kind of relational database or CouchDB or something like that here at these different nodes, which are all computer nodes, but we use a different kind of data structure to do that.

## slide: "Linked blocks instead of list/table"

And this data structure is a **list of blocks** instead of just a table that we have in a relational database.
So we substitute actually the standard databases that you all know, like SQL database or something like that, and we substitute it by this kind of **linked tables**.
And we do that for a very particular reason.
The reason for that is the following:
What we want to achieve is actually to **pack transactions** that we receive into **smaller formats**.
Let's go to the next slide and then we go into the different blocks.

## slide: "Transactions are distributed"

The idea is actually or the need for that is actually that we send transactions now into this network and each transaction is something like A is transferring to B 10 tokens or 10 euros (see figure on slide), bitcoins or whatever you want.
And this information is then being received by the different nodes,
but **it's not received by everybody** because

- we have some kind of network disconnections,
- some nodes are probably down for some time,
- this node is receiving it very early,
- this node is not receiving it because that's a slow line between the different nodes.

## slide: "Transactions distributed – but not evenly"

And the other thing is that this can go further that we have another transaction like D is transferring to E 20 (see figure on slide)
which is **only received by** this node and by that node.

So we have some kind of problems that we need to solve here.

You have the problem.
How do we achieve in this **decentralized approach** where **every node is completely independent**, but just exchanging information about possible transactions
among each other, how can we actually achieve in the end a **status that is shared** among all the different nodes?
If everybody would just put in his database the information that they get, after five minutes, we would have already a complete **inconsistent state**.
Here, if you ask that node what's the account of B, they would say 10 and the account of B is 20.
And if you ask that node, they would say well B and E, they don't have any money. We haven't seen anything about it.
So we need to find a way that we find some **consensus about the transactions** that we have received and that we **all have the same state**.
And that's the magic what blockchain solves, or Bitcoin and also the other blockchains actually solve.

So the first thing is that we have to find **consensus** about it.

And the second thing is also that we have to make sure that the **data that is being stored** here at the different nodes is **not being manipulated**.

And if we go back to that slide, let's say here at this particular state, we all know that X transferred to Y 100.
And that's something that is being stored here in these nodes in this data.
You have to make sure that that **cannot be manipulated**, that any manipulation, for example, by just removing one "0" from that **would be immediately detected**.

And that is something we need to solve.
Now, how do we do that?

## Whiteboard: Blockchain Drawing

Let's have a look first on how we actually work on this different data structures.
Let's imagine now we have these kind of blocks here.
So the first thing is that these blocks, let's say that's block number 100, that's 101, that's 102.
And they have been generated.
That's our blockchain.
That's where the name comes from.
The **chain of blocks**.
The first thing is that we need to **link the blocks** among each other.
How do we do that?
You know this already.
How do we link block 101 to block 100?

student: You save the **previous block** 101?

prof: Yeah, I can save... So your suggestion is we say, okay, we put a link.
And we say we are linked to block number 100.
Any other guess?
Do we really store the number of the previous block or something different?

student: The **hash**.

prof: So we use the hash.
So what we store is actually the hash of block number 100.
Why do we do that?
It means that if we just store 100 and we have the transaction X to Y 100,
this would mean, okay, we just link to that particular block,
but this **link does not** provide any **information about the content of that block**.
If we store the hash of this one, then we connect actually this block to the content of this block.
This is probably what you meant (prof to the student).
And by doing that, we automatically detect, let's say the hash of this one is now,
let's say, to keep it very simple, again, `4711`
and immediately, **if we remove this zero**, what you have learned is that the **hash changes** immediately,
so the hash of that block is no longer `4711`, but different, and then the link breaks down.
So just by checking this, we can check this.

And here is another thing we're really to easily exemplify that.
The locked in **mining demo**, yeah.
So I just have this small demo here.
So we see here we have the block zero.
That's our **Genesis block**.

---

The Bitcoin **Genesis Block** (Block 0) was mined by Satoshi Nakamoto on January 3, 2009, containing a single **"coinbase" transaction** that created 50 BTC. This reward is technically unspendable because it was omitted from the transaction database, serving as a permanent, immoveable artifact. It contained a headline message referencing the banking crisis, symbolizing its purpose as a decentralized alternative.

- **Date/Time**: January 3, 2009, 18:15:05 UTC.
- **Transactions**: Only one, the coinbase transaction.
- **Reward**: 50 BTC (unspendable).
- **Hash**: `000000000019d6689c085ae165831e934ff763ae46a2a6c172b3f1b60a8ce26f`.
- **Embedded Message**: "The Times 03/Jan/2009 Chancellor on brink of second bailout for banks".
- **Unspendable Nature**: The address `1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa` received the 50 BTC, but the transaction was not added to the global UTXO set (Unspent Transaction Output).
- **Delayed Block 1**: The second block was not mined for another six days, emphasizing its role as the unique starting point.

---

We currently have a difficulty of three.
And what we can do now, I think, now I have to shorten it a bit.
Okay, so we just now get another block.
And you see the transactions that are being added are always random.
And what we do now is that we link this block here to the previous one in a way that the previous hash is this one,
`C02`, which corresponds to that particular hash.
And then I get another block.
I just link this to `0089` to this particular one.

So each block actually contains the link to the previous one, but not a number, but as a hash.
And as soon as I change something, and I want to **manipulate** this here, let's say this is 100, you can see.
So immediately we see that the data has been manipulated because the **hash** and the **nonce** and we come to that,
which is similar to the **proof of work** in **Hashcash**, they don't meet each other anymore.
They are wrong.

And also here we see that the link is this one.
So you see the previous hash, what we believe is `0089` and the hash is now `cd33`.
And by that you immediately detect that something manipulated the data in our blockchain.
And that now makes it possible that these kind of blocks or that these nodes can run everywhere.
So we don't need to trust anymore the server itself, because the server delivers us some kind of data,
and we can immediately check if the data is correct.
So that solves the problem of manipulating the data that is being stored.

But now you could argue, but I just could recompute the hash, when this now becomes `4712`,
I can just put `4712` here.
And then I just manipulate all the different links within the block to do that.
So there must be some kind of difficulty to do that.
It must not be easy to do that.
And this brings us now to the point, if we have all the different transactions in our network,
so one thing is that **it must be difficult to manipulate** the data structures here.
It shouldn't be easy, that's one thing.

## Bitcoin: Mining, Hash, Nonce

The second thing that we have to solve is actually **who of these 7 nodes is the one who shall add a new block** to our blockchain.
So the problem is the following. We have now our blockchain here.
And let's say we have just a blockchain of two blocks.
And now we get a lot of new transactions.
We say A to B, 10 and D to C, 5 and A to D, 5.
So these are new transactions that we have to receive.
And now we would like to add this as information to our blockchain.
And everybody wants to do that now. Everybody wants to each node here,
wants now to add this information to the particular blockchain.
And how we do that is by now **creating a new block**.
So we create a new block.
We put the three transactions, one, two, three into the block.
And now we would like to link that one to this here.
So first we take the hash of that one, so we take the hash of 101.
We take a timestamp, take the time, let me write it into our header.
And we need to do something in addition.
Because we need to find a solution that only one is being selected,
or almost one is being selected as the one who is in this network
who can add the block to our blockchain.
So the **problem** we have to solve is, **who is the one who is adding this block** to the blockchain.
So imagine that we are now the network, so we are about 30 nodes.
And everybody of you is now trying to add this block to the blockchain.
How shall we select who the miner or who the node is?

student: The person who finds a valid **nonce** in order to produce a low enough double SHA-256 hash.

prof: That's already the perfect answer. Why shouldn't we use the **oldest one (ie. person's age)?**

student: The oldest what?

prof: The oldest miner. Let's say we are all persons.
So we could all check who is the owner of this particular computer and we use the oldest one.
Why shouldn't we do that?

student: It is **centralized**.

prof: Yeah because it is centralized.

Why shouldn't we do some kind of **round robin**?
So which means that first it's Ivan, then me, then you, then you, then you, then you.
Could also be done because then it's not always the same. We don't have centralization.
We just use...

student: Either it's not dynamic, ie. we need a fixed amount of people and we all know about the participants
or it's like a master so that everybody knows who is in charge.

prof: The problem is that then it becomes **predictable**.
It becomes very much predictable who is going to be next.
What we want to have is that you don't know who is going to be the next.
Such that you can try to manipulate because then you foresee what's coming up.
you try to somehow manipulate it.
You want to have it completely random.
Okay, so another suggestion.
If you do it randomly, I could just think about a random number
and then between one and a hundred and the person who is being closest to that gets it.
Why shouldn't we do that?

student: You yourself are being closest if you want to.

prof: Let's say I'm not providing a miner, but the point is, again, we are **centralizing**.
Again, **you have to trust me**.
I become the central trust point and I do it correctly.

So these are all approaches that are somehow not applicable
if you want to have a complete random solution
and if you want to have a **non-predictable solution**
and if you want to have some kind of trusted solution.
Therefore, we do what you suggested.
We use the **proof of work** that we learned from **Hashcash**.

student: You may not cook a work or pay because if you pay to compute this one or two times
it's the same as you may pay, but not for this.
I'm like one manipulation is already enough to manipulate the network.

prof: Therefore, it **must be very cost intensive** to do it.
Not in a way how I did it here with just a few, let's say one million tries.
So we need to find something where we have really hard work to do that.
And it's the same principle.

student: But isn't it **very inefficient** if you want to use Bitcoin for transactions?

prof: **Yes, it is**.
Yes, it is.
It's inefficient if you compare it to the **amount of energy** that you need.
And therefore, but that's the next lecture, you think about alternative solutions.
But the point is really first to understand how Bitcoin actually works and how this works.

student: If you were to use the Hashcash, when we're going after probability theory, right?
Let's say in total I would need one million tries per average.
Then wouldn't that be the risk of several parties finding the solution at roughly the same time
and then by connections, then yes, you'll get the whole thing?

prof: So let's hold this on, we discuss that.
Yeah, good thing.

But we actually do it now with **Hashcash**, what we say is, okay, now we need to find a **nonce**,
a **number only used once**, that was the number we were computing with Hashcash,
we need to find the number only used once, that **depends on the content of the block**.
So we take our transaction, we take the time, we take the hash of the previous one
and adding all that, so we have the hash of the content plus nonce
and if we hash that
this must be below a certain number, which means it **must have at the beginning** the **number of zeros**.
That's how we interpret below.
And that's our requirement.
And the miner, the node or the miner, they're often called **miners** because you see later on why they're also called miners,
the **node who is doing that first** is the one who is **sending this information that we found in the new block to the network** and then the **network checks it**.
So actually he's putting a **stamp** on that, **like Hashcash**,
and he's putting a stamp that he found a solution to the puzzle and he's sending that to all the blocks.

And let's see, this is how it looks like, also here.
What we did here is actually, here we had the difficulty 3 and we wanted to have, here it's really 3 zeros.
To do that I just reset the blockchain, okay, let's create a new one, okay.
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
Okay, and now if we increase it higher,
it takes longer.
So, you see now we are...

And now you see, if everybody who's doing that now,
if all of you would now start this application,
then you would have one who's first.
**Who would be the first** one at the moment?

student: The one with the **fastest CPU**.

prof: The one with the fastest CPU.
And the one with the fastest CPU will win.
And he will probably win every time.
Therefore, there is a race of just becoming the fastest one.
Which means that **at the beginning** was **just a race**,
really, to have machines that do good hashing.
So there is a particular kind of hardware.
And this particular kind of **hardware is very good in computing hashing**.
So that's actually what you now do.

What other kind of idea?
So the first is everybody of you gets a Dell laptop with the same CPU time.
What would you try in order to become the first?
So we all have the same.
You all have the same software.
We have any other kind of guesses?

student: It's a different approach than the other.
All at the same time.

prof: Different approach could be, all of them, they start with one.
So this program is just counting from one upwards.
However, you probably would say, I **start with 10,000**.
Because if all the other stupid guys start with one,
I start with 10,000 and then they probably become different to all the rest.

student: I mean, **every miner has a different (Coinbase) transaction to their own address**,
which would change the hash and then change all the...,
so **they would all be searching different hash spaces**.

prof: Different hash spaces.
That's something you're right.
**What I said assumes that we all have the same content hash.**
But **that's not the case**.
Because you may have received other transactions than you.
You have received these three transactions,
but you may have received these three transactions is something else.
And again, you have your own address, which is also being involved here.
So it's **your own address that is in here**.
So **it's an address that you also add**.
And there will also be a very special transaction that also makes it different.
So **everybody has actually another content hash for which he needs to find the nonce**.

---

### ChatGPT: "meaning"

What that passage is getting at is a subtle but very important point about how mining works in Bitcoin:

---

#### 🧩 Why miners don’t all hash the same thing

![Image](https://images.openai.com/static-rsc-4/2GIbzgu19u7ywJrkEbPBg4KfKh1p_yEagxh6Rjpu72qhsJzcohKQNvlf76iXpiojIu4QWrNkAI_aS4xY7DKTnEDnUsXF387xP6A9WIKAAXXO1tROl6p8wHyfSxQuBAMXLeD5ZDAUJJcq2vJY7Ar1r6udGOi7BNnGYhN9bh1xPqwyuZ01rZjyd_cqm4SFcx4l?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/ZP1EgErovTeIfguIBulcjayqRUR03UTtCiMo2pQ04nBEj37ag6MokgLFYWl0JP9BwSvRoayXTfIOw88NVtwi4yEr45niOxJhPRBHw9HAQMpnWkbiqAQSkVf0G62v6NUkrFM_hcgBnNQ3xwRjW-TFDUZOi-UzN3I_zK4B4mED-gTtxHx8ytzT-h3jBOgHHyDn?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/Lm2bVPORZThT23F89cGmlbI2_55AuAxIv6IYfGszaRJd6nTOPqqhLo1Q8Mr29jVqMOsF1LyUKwbPvhbtTwgGJmrJr013L6HbfMWXue1mgsoLVexWJFBdWUUpDJ4XmYVyGr2vhY0ZLhdJvEaQLcu_NLvfYhttECN4tKTeOdjqaI7blaBPeBmP7TNPJp3leS1y?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/aWYo_HB8DX_0NA4hqja79WfVsQEvqyhbhY1DpHM_gCcmMcVeF6tfSdW8tGqqrrZAhWcxbA-PKJ_xTo-kPh-RhjyRZsRM-0Y2xOLen9oQ9Xl6p1QDYriqBWK_xVyC9itlifLCGEULxkPdKoUTYpO-0quPRENeCGIVyjOqMNShnl8zoCD_ZegAcZTp-TvTJAos?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/mQnltXwHWSBoSYNhfuQuRdLvGQDATpawzducGRLtHYnkwO7A1M4i4yT-xRQRMkHij630wngkjKYSdieoBYo5Y0aDB1kFotwWSt_fFLjep6AlsBNMrTegGcduhWXc-qGsQOsRsBbrDqejjKFWo5i7gavHCo1e8-37HQFZ5lCpbg2mdwCVckgLoX7pKLkAh9mG?purpose=fullsize)

Even though everyone is working on “the next block,” **they are not hashing identical data**.

##### 1. 🧾 Different transaction sets (mempool differences)

Each miner has their own view of pending transactions:

* You might have seen transactions A, B, C
* Another miner might have A, D, E

👉 So the **block contents differ**, which already changes the hash.

---

##### 2. 💰 The coinbase transaction (miner’s reward)

Every block includes a special first transaction called the **coinbase transaction**:

* It creates new bitcoin (block reward + fees)
* It sends that reward to the miner’s own address

👉 Since each miner uses **their own address**, this transaction is different for everyone.

---

##### 3. 🔗 Result: different block headers

Because of:

* Different transactions
* Different coinbase transaction

→ the **Merkle root** changes
→ the **block header changes**
→ the **hash target puzzle is different**

---

#### 🎯 So what does the statement mean?

> “Everybody has actually another content hash for which he needs to find the nonce.”

It means:

👉 Each miner is solving **their own version of the puzzle**, not a shared identical one.

* The *difficulty target* is the same for everyone
* But the *input (block data)* is different

So:

* Miner A searches for a nonce for *their* block
* Miner B searches for a nonce for *their* block

---

#### ⚡ Why this is important

##### ✔️ No duplicated work (mostly)

Miners aren’t all hashing the exact same block → less direct overlap

##### ✔️ Competition still works

Even though inputs differ:

* Whoever finds a valid hash first wins
* Their block gets accepted by the network

##### ✔️ Flexibility

Miners can:

* Choose which transactions to include
* Prioritize high-fee transactions

---

#### 🧠 Simple analogy

Think of it like a lottery where:

* Everyone must find a number that makes their ticket valid
* But **each person has a slightly different ticket**

👉 Same rules, different inputs, same race.

---

If you want, I can go one level deeper and explain **how miners still end up converging on one chain despite working on different block versions**—that’s where consensus really kicks in.

---

prof: So you could say, well, I'm not starting at one,
because I think starting at one million is probably better,
because then the other hash can be found easier.
I have no idea.
But you can do some kind of guessing.
But there are other ways to do it.

student: Isn't there like the **conflict** that if we make it **too easy**,
too many people would find it at once,
and make it **too difficult**, no one will ever find it.

prof: And you see here, my computer is now shortly before burning.
It's now trying to find, specifically, seven, it's really already working a lot.
About this, we also discussed that.
I mean, these are all features that are built into Bitcoin,
which are not problems, but actually features.

Okay, so let's compute.
But there is still one way that you could do as a group to become faster.

student: Like a **mining pool**, you mine together.

prof: Yeah, you're going together.
So what you could do is that you two say,
okay, we get the same data,
we become one node to the outside,
but internally we have two computers,
and you start at one, and you start at one million to find it.
Which means that you now have two computers trying to find the nonce.
Obviously, you use not the nonces that start at one, because that would not make sense,
but you **start at different nonces**, and then you use that.
And then you two have almost the double power than all the others,
which means that you will then be almost the first node.
And that's then a mining pool.
You just put together all your computing power,
and you distribute the solving of the puzzle to all your computers that you have available.

student: Okay, maybe I didn't understand a part of it.
So like, we have two computers,
but doesn't it just mean that they're looking at the same time
in different spaces to find a solution?
But not necessarily they're going to find first,
but they can, because its probability is a bit higher,
but **if they have two slow computers, one very fast computer can beat them**.

prof: Definitely, yeah, that's the case.
So if they have two slow laptops, and you have a very fast one,
then you probably overrun them.
But nevertheless, it may be that most of the time it's the others.
And that makes it so interesting.
You can't really predict what's going on.

student: Maybe we'll talk a little bit, but right now I do not understand
**why do I want to be the first** one to find a good one.

prof: Right.

student: From my perspective, everyone gets the same data eventually,
so every data will land on the blockchain.

prof: Yeah, and this question is something that we discuss now.
So it's really the point why.

## slide: "Mining Process"

The point actually now is... This is the **mining process**.
That's the whole mining process.
It's so easy.
It's **just a loop** that we do, we

- **hash the block header plus the nonce**,
- and then we **hash it again**,
- and then we see, **is this below the difficulty (ie. number of zeros in front)?**, so 4070 was what we discussed here.
  - **If no (ie. not enough zeros in front)**, add one to the nonce, and
  - **if yes (ie. enough zeros in front)**, send the block to all neighbors that you have available, and this is what's happening now, that you send all the blocks to your neighbors with the solution of the crypto puzzle, and then what they do is they just check it, and that's easy. They just need to check is this hash of the content plus the nonce in total has the certain amount of zeros. Fine, then that is being accepted.

## Bitcoin: Block Reward, Transaction Fees

So now we come to some questions.
One question is **why should I do that?**
What's my incentive?
What's my benefit?

student: You get the **block reward**.

prof: You get a block reward.
You get money.
But how do you get money?
Where do you get the money from?

student: In addition to the block reward, you also get the gas and mining fees.

prof: No, gas is something we discuss when we talk about Ethereum.

student: But **transaction fees**.

prof: Transaction fees, yeah.
Transaction fees, yeah, that's what you get.
First thing is that a transaction within these kind of blockchains,
they don't do it just for fun.
They want to earn money.
It's not for free.
A blockchain transaction is not for free.
If I want to send you some coins, they want to send you a token, some crypto or whatever,
I have to add fees **like with a bank**.
You also pay.
I mean, this is something where very often people think, well,
Bitcoin transactions are for free.
That's what differentiates it from bank transfers.
No, they cost you money.
And very often it's **calculated** in an **amount** that **depends on the current traffic in the network**.
If the Bitcoin network or if the blockchain network is busy,
you pay more fees than if it's not busy.
This is Uber.
You all know that when you want to go on a Saturday evening out of Aachen, back home,
Uber is expensive and on a Monday morning it is cheap.
So the **mining fees** really depend on the traffic in the network.
And what these people do here is that when they get all the transactions,
**they select those transactions with the highest fees**,
which means that when you submit a transaction into this network and you don't provide any fees,
you could do that.
You could say, please, I would like to send you one Bitcoin, but **I don't provide any mining fees**.
**What happens?**
This **will not be included**.
All the miners, they want to have transactions with high fees.
So they probably just put it aside.
And only if there probably is no traffic at all,
they probably put it into their list of transactions.
But maybe never.
But it could be taken a day or a week after the transaction.

## Bitcoin: Number of Transactions per Block

student: But is there **for every block just one transaction?**

prof: No, every block has...

student: ~3,000 transactions. (Gemini: As of March 2026, the **average number of Bitcoin transactions per block** is approximately 3,300 to 3,400.)

student: Okay, but it's a fixed amount.

prof: It's **a fixed amount**.
That's being discussed, always discussed,
within the community, it also depends on the blockchain.
At the beginning, there was not that many transactions, but they're now increasing it.
So for the whole process it doesn't make much difference.

student: Yeah, because otherwise you could just put every transaction in.

prof: Yeah, but they also want to get money.
Yeah, if one miner probably on every transaction...

student: It's like limiting of the people that spend money on blockchain.

---

### ChatGPT: "what is the role of the number of bitcoin transactions per block? what changes when this number is changed?"

The “number of transactions per block” in Bitcoin isn’t fixed directly—it’s a consequence of the **block size limit** (≈4 MB weight units) and the size of individual transactions. But thinking in terms of “how many transactions fit in a block” is still very useful, because it affects how the whole system behaves.

---

#### ⛓️ Blocks and transactions (quick intuition)

![Image](https://images.openai.com/static-rsc-4/4vDxnCh5cND6kd2x4lJt4esocgh_96uj3xriNTubAvhGfBaG6A3zkhGXhSiYtwlVjNBRrxXPFUExKvXsoOEO6-zcBxHKT6EUEJneL76_RzMhoDWu7D-8Ofwd7GxgAyGUZpUzBJY00AmxiVxzan8dHcw40QFP9ZlOqS-6kNYEBn7uViyGeAdznO3K4s9bfipl?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/Fr_rTcvPICcucNBjBHMP-tfDaOaOEBF6SBILS9yXZ66cbBQB062NRuBi9t3xdBiFvam65SURTt-KuPztJ6O9c2I8DT-f7Tnw1gbEIW7P-6mW1-ipYYbDq4age4vW01od_FiUleEzyT1FNkLNqgD0wjtp7Ip-V0J59nHpzT_K6vf-re17vQgvuHIVEN0redZj?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/mQnltXwHWSBoSYNhfuQuRdLvGQDATpawzducGRLtHYnkwO7A1M4i4yT-xRQRMkHij630wngkjKYSdieoBYo5Y0aDB1kFotwWSt_fFLjep6AlsBNMrTegGcduhWXc-qGsQOsRsBbrDqejjKFWo5i7gavHCo1e8-37HQFZ5lCpbg2mdwCVckgLoX7pKLkAh9mG?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/LPDmtPtm_k2H3CZ4R_bUYlBnI1oj0egGiyG72Tg6pfOe4JEom6qZbXJX2pv7PJendGI74IY86Ap6haf4P7Pxu9UvoL6UaY5Z4viY5-lBBlWkuUSauPsKA-F8vjhhlKKC73THJY__xAFdrEoXWx104Zfr_m_Qlz-clRuCraeuvVwS-QRomxrcr77MwLG-z6Iy?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/FchRk6raOdQu1tlAKkV1dwUX-BB15IEDDp8vVnWHj-Sw0UDawN7iAOiwCs9G0zGRQVVqM4WFfDQEIpPaXgEiz9jW5qRls1PX9fXlnjyPyVF0Mgi17PWqiYHDrAXxRv9hOjZVk4qwLuVmXqVT3lmmUiTYcffRetQC16b_MvPuo0CVkr6ub58o6qCp8PV3fHC6?purpose=fullsize)

Every ~10 minutes, a new block is added. That block can only include a limited number of transactions.

👉 So the number of transactions per block determines how many payments Bitcoin can process every 10 minutes.

---

#### 🚦 1. Throughput (transactions per second)

* **More transactions per block → higher throughput**
* **Fewer transactions → lower throughput**

Example:

* Small blocks → maybe ~3–7 transactions per second
* Larger blocks → more transactions per second

👉 This directly affects scalability.

---

#### 💸 2. Transaction fees (supply vs demand)

Bitcoin has a **mempool** (a queue of pending transactions).

* If blocks are small (few transactions):

  * Many transactions compete for limited space
  * Users bid higher fees → fees go up

* If blocks are larger (more transactions):

  * More space available
  * Less competition → fees go down

👉 Block capacity = “supply” of transaction space.

---

#### ⏱️ 3. Confirmation time

* If blocks fit **more transactions**, your transaction is more likely to get included quickly
* If blocks are **full**, you may wait multiple blocks unless you pay higher fees

👉 Bigger capacity → shorter waiting times (on average)

---

#### 🌐 4. Decentralization vs. centralization trade-off

Here’s the tricky part:

* Larger blocks (more transactions):

  * Require more storage, bandwidth, and processing
  * Harder for individuals to run nodes
  * Risk: fewer nodes → more centralization

* Smaller blocks:

  * Easier for anyone to run a node
  * Stronger decentralization
  * But limited capacity

👉 This is one of Bitcoin’s core design tensions.

---

#### 🔒 5. Network security and propagation

* Bigger blocks take longer to spread across the network
* Slower propagation can:

  * Increase orphaned blocks
  * Slightly weaken consensus efficiency

👉 Smaller blocks propagate faster and keep the network tighter.

---

#### ⚖️ What happens if you change it?

##### If you increase transactions per block:

* ✅ Higher throughput
* ✅ Lower fees
* ❌ More centralization pressure
* ❌ Slower block propagation

##### If you decrease it:

* ✅ Stronger decentralization
* ✅ Faster propagation
* ❌ Higher fees
* ❌ Lower throughput

---

#### 🧠 Big picture

The number of transactions per block is really about **how Bitcoin balances three things**:

* Scalability (more users)
* Decentralization (more nodes)
* Security (robust network)

You can’t maximize all three at once—changing block capacity shifts that balance.

---

If you want, I can connect this to the **Bitcoin block size wars** and explain why this became such a controversial topic.

---

prof: So, first thing is they get **transaction fees**.
And they get a reward.
They get a **block reward**.
So what they do is...
And that also makes **every content different**.
Even if you all get the same 3 transactions here,
you all have your own ID, which is being stored in the block header.
And there is **one particular (coinbase) transaction** where you say,
just one, let's say one Bitcoin, BTC, **to me**.
So you just include one transaction.
Then you put one Bitcoin to your own wallet.
So you just create this kind of Bitcoin out of nothing.
It has never been there.
You just say, my account has now plus one Bitcoin.
And at the beginning, when Bitcoin was launched **in 2009**, that was **50 Bitcoins**.
So for every block, you got 50 Bitcoins, which would nowadays be 50 times 100,000, 5 million.
That was 2009.
So 50 Bitcoin, every block.
That means with every block that we add to the blockchain,
the **money** that is **available** is being **increased by 50 Bitcoins**,
which is quite some **inflation**.
It's like our Bundesbank or the European Bank, who is constantly printing Euros.
**We print Bitcoins** here.
But only **every 10 minutes** (Gemini: The network typically generates one block every 10 minutes.).
The 10 minutes is something that comes into the game.
That is also something.
We don't want to have inflation too fast.
We want to have inflation in a calculated way.
So therefore we say, a block should, among all yourself, be found within 10 minutes.
And the difficulty is therefore adapted such that the community of all miners finds a block on average every 10 minutes.

student: But then in the case of **Bitcoin**, the **block reward gets halved every 210,000 blocks**.

prof: That is something I would like to come to.

So let's see what our miner is doing.
He found it.
462 seconds.
And at difficulty 7, the nonce is 185,808,000.
So it took him 46 seconds, which is about 7 minutes.
Oh, we are quite there.
So for us, the difficulty of 7 would mean that we can be in the game.
But I think the current difficulty is about 14, or 15.
You see how dramatically it increased really to find this nonce.

So what we want to have is a block every 10 minutes.
And with this particular block, we create new tokens, **we create new Bitcoins out of nothing**.
So **we increase the number of Bitcoins that are available**.
And this should be done almost every 10 minutes.

## Bitcoin: Mining Difficulty Adjustment

And if you all now invest in computing power, and suddenly you find the block every 5 minutes,
what do we do?
We **increase the difficulty**.
And by that, we bring it back to 10 minutes.
I think this is extremely elegant.
You just increase the difficulty, you come back to 10 minutes.

Suddenly, you all say, well, now I'm doing large language fine tuning.
So I hate all this Bitcoin stuff.
I put my computer now on fine-tuning and large language models.
So suddenly, we find blocks only every 20 minutes.
We **reduce the difficulty**.

So **we just adopt our difficulty to the current computing power that is available**,
the current computing power that is available **in the network**.

student: There is certain harm in it.
So not really like miner points and stuff, but like the step of 67 was of time factor, I think, 70.
So what is there is no probability to reduce it from 20 to 10,
but if you reduce it by 1, you already get 5.

prof: So you mean that, if you just increase it from 100 to 101,
that nobody would find it anymore because the solution space is so small.
That's interesting.
Could be, could be that this happens,
but then they probably use some other kind of difficulty measure.
So something they have created.

student: It's **an average over the last 2016 blocks**,
and it **can only be adjusted up by 4 and down by a factor of 4**.
But depending on the average, it can go up also a very, very small amount,
or down a very small amount.

---

### ChatGPT: "meaning"

This exchange is about how **Bitcoin automatically keeps block production close to ~10 minutes**, even when total mining power changes.

---

#### ⚙️ Difficulty adjustment in Bitcoin

![Image](https://images.openai.com/static-rsc-4/lEmkGfsW4hnpErJ02-_H43-F5nRhmUFqH75cqZ6D6l6MaFdt4DZdBIBIjxbag_vpr8C9Aak8UFJSo8cUh6Kzn6LUDkfUSiI471HcZXsHSp3mt6g7qpg24qhNq2sweZbJwUZhkdpSuXoAeDENZ7Cis2JR_DCvmdUgmYnG2-WJOl1JKudnjN4DQIVPrGFEGA07?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/wsyN1xf1jGFu5IBISnGnIfAgyoI9uGRWg10vmYnddKNckwe3LU7g8wwdJZV-s71D11Ju2ntqiAsLEcGZmPix42r-y8A6oLDgqm5HudgVfIIsP6GfaQTsd519FUm6snAbMfbogDBEPhgG6qoqHIb89EnEQkXZfRGEF-t0Unt9nZ0dJFbpT1go4mNwtWXhjhFP?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/oDU9lxdcxAOrdZWYUl61l9gI5fzAHoGKDv21RPCR0FrDnSCIJ_QSC4-k4SA6_wXSFxbGQAWxY2ctxsIaOjnedv9GdEMiPYSi7L5ePg-6VmmlL-HqxAPXHe9AR3O5pDlHpWiTB2vkoZJCbPzd2DxWSsI2PaOfgskZDPrtFw1rGsIfpMLnvLFk-RdNj8-6BbYs?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/72422FaWOhetiCAn_wkXQ04tNb0IQWjAal7cT4zBfAOigKtNMqzLu5QUA_qltH_TjOWSLeANRX_jAcEgYYxuR_TloBf8E6HniVIskGGorH12ZzvhCH-6NEoKJ2K1yLV6orKUZTH9e9fBomoZK3GQMlE09G51GDeXxichW6YKS0NPBgoGFK2QlRGbUT9tloco?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/6Ysy3kfMNgJ0sFbiPpYK3srYHBRJTtrRacS2BDv9GxrYErrM7pggoQLqgy98bb_45rEpfUl9M5appRY_gvVDQBQPR_v-OITgiNNxS_vVsiALdHJFF8nYVmAOWXHlHVaCqTlKeRA8E4FGUon6h7H-dR8TFpMjwFhnrLdshzQhFpCuArvGFdIQ_r3RtowP0Pe6?purpose=fullsize)

##### 🧠 Core idea

Bitcoin doesn’t fix how *hard* mining is forever.
Instead, it **continuously adapts the difficulty** so that:

👉 One block is found roughly every **10 minutes**, no matter how much computing power is in the network.

---

#### 🔄 How the adjustment actually works

##### ⏱️ Step 1: Look at the last 2016 blocks

* 2016 blocks ≈ 2 weeks (because 2016 × 10 minutes ≈ 14 days)

Bitcoin asks:

> “How long did it *actually* take to mine these 2016 blocks?”

---

##### ⚖️ Step 2: Compare with the target

* Expected time: **2 weeks**
* If it took:

  * **1 week** → blocks were too fast → increase difficulty
  * **3 weeks** → blocks were too slow → decrease difficulty

---

##### 🎯 Step 3: Adjust difficulty proportionally

The new difficulty is scaled based on: $$\text{new difficulty} \propto \frac{\text{expected time}}{\text{actual time}}$$

So:

* Faster mining → harder puzzle
* Slower mining → easier puzzle

---

#### 🚧 Important constraint (student’s point)

The student is correct:

👉 The adjustment is **bounded**:

* Can increase at most **×4**
* Can decrease at most **÷4**

This prevents extreme swings that could destabilize the network.

---

#### 🤔 About the professor’s question

> “If you increase difficulty slightly, could it suddenly become impossible?”

This reveals a misunderstanding.

##### ✔️ Reality:

Difficulty is **not a small discrete step like 100 → 101**

Instead:

* It adjusts a **target threshold for hashes**
* The hash space is astronomically large (2²⁵⁶ possibilities)

👉 So:

* There are always *many* valid solutions
* Increasing difficulty just makes them **rarer**, not nonexistent

---

#### 🎲 Intuition: searching in a huge space

Mining = trying random hashes until one is below a target.

* Easier difficulty → large target → many valid hashes
* Harder difficulty → small target → fewer valid hashes

But:
👉 The space is so huge that you never “run out” of solutions.

---

#### 🔁 Putting the lecture together

The lecturer’s point:

* If more computing power joins → blocks come too fast → difficulty increases
* If miners leave → blocks slow down → difficulty decreases

The student adds:

* This adjustment happens every 2016 blocks
* And it’s smoothed + bounded to avoid instability

---

#### 🧠 Why this is “elegant”

It’s a **self-regulating system**:

* No central authority
* No manual tuning
* Automatically tracks global computing power

👉 Bitcoin behaves like a thermostat:

* Too fast → cool down (increase difficulty)
* Too slow → heat up (decrease difficulty)

---

If you want, I can show you **the exact formula Bitcoin uses (with the target and bits field)**—that’s where it gets a bit more technical but very insightful.

---

### ChatGPT: "what is 'target'"

Great question—**“target” is the key piece that makes Bitcoin mining work.**

---

#### 🎯 What is the “target” in Bitcoin?

The **target** is simply a **number** that sets how hard mining is.

A block is valid only if:

👉 So the target is a **threshold**:

* If your hash is **below it → success**
* If not → try again with a different nonce

---

#### 🔢 What kind of number are we talking about?

* Hashes in Bitcoin are **256-bit numbers** (from SHA-256)
* That means values between:

  * 0 and ~2²⁵⁶

👉 The target is also a number in this same range.

---

#### 🎲 Why the target controls difficulty

##### Big target (easy)

* Example:
  “Any hash starting with `0000...` is fine”
* Many hashes satisfy the condition
* Blocks are found quickly

##### Small target (hard)

* Example:
  “Hash must start with `000000000000...`”
* Very few hashes qualify
* Takes much longer

👉 So:

* **Smaller target = higher difficulty**
* **Larger target = lower difficulty**

---

#### 🧠 Intuition: lottery analogy

Think of:

* Each hash = a lottery ticket

* Target = winning cutoff

* High target → many winning tickets

* Low target → very rare winners

---

#### ⚙️ Where does the target come from?

The Bitcoin protocol:

* Recalculates it every **2016 blocks**
* Based on how fast blocks were found

If mining was:

* Too fast → **target is lowered** (harder)
* Too slow → **target is raised** (easier)

---

#### 🔐 How it looks in practice

In real Bitcoin blocks:

* The target is stored in a compact format called **“bits”**
* Nodes expand it into the full 256-bit number

And hashes look like:

```
0000000000000000000a3f... (very small number)
```

👉 The many leading zeros are just a visual clue that:

* The number is **very small**
* → below the target

---

#### 🧩 Big picture

* **Target** = the actual rule
* **Difficulty** = a derived measure humans use

👉 Mining is just:

> “Keep hashing until your number is small enough.”

---

If you want, I can walk you through a **real Bitcoin block example** and show exactly how its hash compares to the target—that makes this concept click instantly.

---

prof: And you see here, I'm using now seven bytes.
If you do it in bits, then it becomes much higher, right?
With Hashcash, I do it in bits, but here I do it in bytes.
And a byte is just 8 bits that you need to find.
So the solution space is, every number becomes really small.
It takes some time to provide the right answer.

## Bitcoin: Halving Mechanism

Okay, so we now, we learned, we do it because we get money.
We get bitcoins as a miner every time.
We increase the amount of bitcoins that are in the market.
We have some kind of **inflation**,
and if you already have too fast with the computers,
we just increase the difficulty.
Nevertheless, it means every 10 minutes, 50 bitcoins.
That's too much.

At the beginning, we wanted to have a lot of coins immediately in the market.
And we really wanted to flood the market with bitcoins so that we get some kind of, just a big market.
After some time, this becomes problematic.
You need to reduce it.
What we do is, we just **halve the amount of bitcoins that you get for the mining**,
from 50 to 25, from 25 to 12.5, from 12.5 to 6.75,
and from there to 3. something which is the current amount that you get.

And you sometimes get these kind of spam emails,
Bitcoin halving, buy Bitcoin, Bitcoin becomes more valuable.
That's the Bitcoin halving.
**They don't halve bitcoins. They just halve the amount that the miners get**,
which means that actually, at the moment, it's 3.75,
and then after a certain, I think, yeah,

student: 3.125.

prof: 3.125, BTC with everyone.
And next time, when we do the halving, that is 1.6 something.
And **we do it every 210,000 blocks**, as I showed you.
So we do it every 210,000 blocks,
which means that if you divide it by 10 minutes,
it's every two years or so.

student: **Every four years.**

prof: So every four years, you do the halving.
And the **last one was** what, one and a half or two years ago.

student: **April 2024**.

prof: Yeah, I agree.
You see, I'm not dealing with this event so much.
So that's something that will happen in the next time.
So the **next one will be in**...

student: **2028**.

prof: 2028, yeah, so 2028, still great.
So in two years, almost.
And then...

So, but if we halve and halve and halve and halve,
**after some time**, there will be no incentive,
there will be **no block reward**.
How will the system then still be running?
Why should people...

student: ... transaction fees.

prof: That's the idea.
The idea is that then, and this will take place in year 2000...

student: **2140**.

prof: Yeah.
2140.
That's **the year when there will be no more block rewards**.
So still more than a hundred years away.
And until then, they just speculate,
or the inventors speculated back then that **this will just be based on transaction fees**.

I don't know, I just wanted to look this up.
Could be a nice task for you to look this up.
What is the current **ratio between block reward and mining fees**?
It could already be that we soon reach that miners get more transaction fees than block reward.
Because there are almost 10,000 transactions in there (March 2026: about 3,300 to 3,400 transactions per block, see above).
And imagine that there is someone sending five bitcoins and if you get them, one or two percent as a transaction fee,
you would get already quite a lot...
This is why people do that.
Actually.
That is why you can still start mining.
You wouldn't have much luck, you see, with this example here.

student: What stops the miner from putting more bitcoins in itself while mining the block?

prof: Yeah.
If you put in 10 bitcoins to me,
and you send that block to other miners, what will they do?

student: Reject it.

prof: They check it and reject it.
You are just playing false and probably they will just blacklist you.
From then on, you are gone.
You have to find a new IP number then.
Because they blacklist you.
So that's something that wouldn't work.

---

### ChatGPT: "meaning"

The exchange is getting at **why miners can’t just “pay themselves extra bitcoin”** when creating a block in Bitcoin.

---

#### ⛏️ The temptation: “why not give myself more?”

![Image](https://images.openai.com/static-rsc-4/_lhq0SVYjG--M9IlkeELEzJV5QpncryLdViivYS897GkxLorXXgvJwnK0cUMYa5QbnY87MBErJ1o6tE8SOGCUsMWcFFQMZ9VIkVHQxqyhVnIqfIuCcz5OTgBL4JIvPjUcKb-CVxNeaNsLmrlCai10S6lUiTdJTSeqwfk_-gWrzWCpFCLD35m7EbFeVs560Qk?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/wwC6qV_KBy8X7TqM7k3R3otCI3SSMU6yadN0BMj9w8n1USK6TI1nsf6xSNuTtn0l28v-_Xy1aW7UcdedrUQSnHDUG4rx7y4AkeGb4CccSgI4In74CzLx4-9G0jRRYS7CXlGqugX6cySYAWg80hJi5nqxAxwtwgrdKKXlK7XPWcBfx646gk0ZOKZWIUkibUor?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/HMfx2BKN9nFHtmEGLZhLVyfe4-eYfDkLxptkcoVeyTHaGxflyWl2Ssk0lwgLejYH_HHVQplN7uyHRxgfVEchkVI9KhL2FKyJF9GL1sHo9PtvC3LYatD1_wXx3HHBgpEo_cBp4eCh1mnYdwvXdFObVH9lzfM7JSq_1sIDN0UkOpEl7vbnN6MJI8Fqrif6n4-P?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/MjrPKyL_YPVNO-aMvON-1IlnnEvnXIWFvso7MBbAxQsrXQPBd3C4lKJoHzEDUZpqFrO6FQK8kiaSDNU2_fFvakkT2HniYX0a5azzqN897wEMwp-EWIbLghce0-TtYOhAMZOggCrf53bvHhwPUltnjsxvErjDLvwyRMx8AxNTFDeirXzgK1r-Nn2Jw6eqChiX?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/0kqYnQ-u-9SyVyKFMKyX_9TvUBo2NKviGzr-oTUAQGYUVy3ITFdEVSTk2D177RXiX-ereIYkjqBgkJefbvLt0Ar8Io0RZhGgmv3Kn1Yaqp6uMWRR9iQa6Z5dLflH-y1deYPQ0568yJaB-s9yNlF_1y2BppY8Coq4Utb1ai9PaKbH8sizq8UraGNHKszGwTQj?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/K2MAAqxgC-i7962MU6Kyxkf9A38mJ_a_NE_IIrVdGiuNLPFnoSZYU8Dy1N5e-I2aXtTlJIXb-PIYe-LWpjGefBILCVR8r_wnTheQptgrFy1uoIfFC9JFhNzacuUbC0M3l3foY3VySm3UERaVAN1yswSNG0N-otiFLy9syI12a0uxGccvRgXunFOdVDx6_EMK?purpose=fullsize)

When a miner creates a block, they include a special transaction:

* the **coinbase transaction** (their reward)

So the student is asking:

> What stops a miner from just increasing that reward arbitrarily?

---

#### ✅ The real answer (core idea)

👉 **All other nodes independently verify the block.**

If a miner cheats:

* The block violates the protocol rules
* Other nodes detect it instantly
* The block is **rejected**

---

#### 🔒 What exactly is checked?

Every full node checks things like:

* Is the block reward correct?

  * (fixed subsidy + transaction fees)
* Are all transactions valid?
* No coins are created out of thin air?

👉 If the miner gives themselves even 1 extra bitcoin:

* The block is **invalid**

---

#### 🚫 What happens to a cheating miner?

* Their block is **ignored** (not added to the blockchain)
* They **waste all their computational work**
* They get **zero reward**

👉 That’s the real punishment—not “being blacklisted.”

---

#### ⚠️ About the professor’s “blacklist” comment

That part is a bit misleading or simplified.

##### In reality:

* Bitcoin has **no global blacklist mechanism**
* Nodes don’t permanently ban miners for one bad block
* There’s no identity system like “this miner is banned forever”

What *can* happen:

* A node may temporarily disconnect misbehaving peers
* But the main consequence is economic: **you lose money**

---

#### 🧠 Why the system is secure

This works because of **incentives + verification**:

##### ✔️ Verification

Every node enforces the same rules independently
→ no trust in miners required

##### ✔️ Incentives

* Honest mining → reward
* Cheating → guaranteed loss

---

#### 🎲 Simple analogy

Imagine submitting a winning lottery ticket:

* If you try to **forge the numbers**,
* The lottery system checks and says: ❌ invalid

👉 You don’t get paid—you just wasted your effort.

---

#### 🧩 Bottom line

The key idea behind the dialogue is:

> Miners can propose blocks,
> but the network decides if they are valid.

So:

* You *can* try to cheat
* But the system makes cheating **pointless and costly**

---

If you want, I can break down **exactly how the block reward is calculated (including halving and fees)**—that’s the precise rule miners must follow.

---

## Summary

So I think what we learned now is how this is all being organized.
So we have these kind of blocks.
We learned why we do that.
The incentive for doing that, to invest money and computing power into that:
because you get a block reward - beyond probably just the idea of I would like to be part of the community because I believe in Bitcoin and cryptocurrencies also.

## org

And one thing that you also should know about is this...
And then we can stop because that's probably then a good break.
And then **next time** we discuss **what happens if we have two blocks being found**.
**How the actual transaction works**.
But this would open up now too much.

## slide: "Hash-Funktionen and Merkle Tree"

**Merkle tree**. Do you know Merkle trees?
I mean it has nothing to do with our old chancellor.
Same name but it's not named after her.

The **Merkle tree is the way we organize the transactions in a block**.
So I was actually cheating here.
I said, well we have a **header**.
And below that we have a **list of all the transactions**.
This is **not what we do**.
We do it in a different way.
So **what we actually do** here is that we have this kind of **(Merkle tree) organization of each block**.
So each block has

- the **previous hash**. That's what we have.
- We have **timestamp**, also cool.
- We have the **nonce**.
- And we have the root of the Merkle tree (**Merkle root**).

And you see we are not listing our transactions just in a list.
And we hash that.
What we do is we take transaction number zero.
So we have four transactions in this example here.
Zero, one, two, three.
And then we take the hash of transaction number zero and the hash of number one.
We have two hashes.
We combine that into a hash.
And do the same here in our tree.
And then actually we **hash these pairs against each other**.
And **that gives the Merkle root**.

So actually some miners which are called the **light nodes**,
these light nodes, they don't contain all the information here.
They just contain that.

---

**🪶 Light node (SPV)**

SPV node = Simplified Payment Verification node

* Runs on phones or lightweight apps
* Doesn’t store full data
* Asks "full nodes" for information

**🪶 Lightweight verification (SPV nodes)**

“Light nodes” don’t store the full blockchain.

They can:

* Verify their transaction is included
* Without downloading everything

👉 This is crucial for scalability.

**🪶 Light nodes (SPV)**

* Do **not store block bodies**
* Only store headers
* Request data when needed

---

So they just work in the network.
And they just work in the network.
And they do some kind of **proof that the new node is correct** or things like that.
But **they don't contain all the information** because that information is quite large.
Downloading the whole Bitcoin blockchain, so really the data, is gigabytes.
I'm not sure, 900 gigabytes or so.

student: **800 gigabytes**.

prof: 800 gigabytes.
That means that if you want to now become a miner to download the software,
it takes a minute.
Installing it, probably just a minute.
Downloading 900 gigabytes takes days.

So although, but you could then run something like a **light node**,
or the lightning network or things like that,
that **could be run even on a Raspberry Pi**.
But that's something we will probably discuss later on.

So how we actually organize this is that within the core data we just have one hash.
And that's the hash of all the transactions that is combined as this Merkle tree.

And when I saw this first, I thought why?
I mean, why do we hash all these pairs and then we hash the pairs
and put it there.
We could just hash that and put it there.
Do you have any idea why?
**Why do we use this kind of tree?**
I mean, computer scientists, they love trees, obviously.
Yeah, so we do everything in trees, but the list and then hashing it?

student: I was thinking maybe we can, using that somehow find out if somebody cheated where he cheated.

prof: No, no.
Any other guess?

student: To avoid like going over the side of the block, having this one hash.

prof: Yeah, but that's the same.
If we, why don't we just hash this pair-wise and we put the hash in there
or just taking that and put the hash in there.
That's almost the same.

student: That can work with subtrees.

prof: Yeah, it goes into the right direction.

student: If I manipulate transaction one, the hash to three would still be correct.
I could identify which way I'm going to go.

prof: Yeah, it goes also in the right direction.
Yeah.

student: It allows also miners to change the **Merkle root** slightly.
In case, for example, they run out of **nonce space**,
you can also change a little bit the Merkle root and then go over a new **hash space**.
I'm not sure that's the answer you're looking for.

prof: No, I didn't get this.
I'm looking for another solution.

The point is that what happens **if you want to check if a certain transaction is actually within**..., this example here,
so we store all the transactions and we just hash everything and put this into our, this is `Tx_Root` (see figure on slide).
So we have all the transactions and we hash it and we get it into `Tx_Root`.
And we have that one.

**If I give you now a transaction**, I give you transaction number one, which is this one.
This is `Tx0` and this is `Tx1` (see figure on slide).
I give you transaction number one.
**What do you need to know if this transaction is in this particular block, if I give you also the, if you know the `Tx_Root`?**

So the question is the following:
I give you a transaction and I claim that this transaction is actually being in block number 11.
**Which kind of information do you need to know to check if transaction number one is in block number 11?**
In this example.
Let's take this example first.

student: Do the math to do the reverse hash.

prof: Not the reverse hash but...

student: ... like the bit mask to ...

prof: This is not, they are not concatenated to identify them.
They are just hashed and hashed and hashed.

student: All the transactions.

prof: **All the transactions**. Yeah, in this case here, I need all the transactions to check if transaction number one is here.
So I need 1000 transactions or **in this case, let's say, I need 4 transactions**.
Let's say, just a 4.
I need 4 transactions to check if transaction number one is in this block.

How many other transactions' information do I need to know if transaction number one is in this block **using a Merkle tree?**

student: ... subtree ...

prof: Subtree, which is I need transaction number 0 and
I compute this, I compute that and I need the hash...
I don't need this and this, so I need 1, 2, 3.

student: The **hash of 0 would be already enough right?**
The transaction 0 could be anonymous?

prof: Yeah, **but this is not stored**.
The only thing that is actually stored in the node is this (the 4 transactions).
That's the interesting thing and that also gives these **"light nodes"** some kind of sense.

If I claim that my transaction is in block number 12 and this node doesn't know all the other hashes,
I just need to provide my transaction, transaction number 0 and that can be computed if this is the same as this one.
So, normally I would need to provide all the four transactions.
Here, I just need to provide 3.
Now imagine that you don't have such a simple Merkle tree.
We have a much larger Merkle tree where we have much more nodes.
Then again the information I need to provide to check that a transaction is in the Merkle tree is normally only the number of levels of the tree.
You automatically get up and you just need to provide the pairing hashes and that then reduces computing power and data that you need to provide.

student: How do I know in this case for example hash 23 is in the root stored?

prof: This is then often stored here within the nodes themself.
They know that.
They need to do less comparisons.

student: This is the header.
Then the block itself really contains all the data.

prof: For example, if one node wants to communicate that to another node,
then it just needs to send this, this, this and that.

Actually, Merkle trees they have been invented before blockchain.
They have been invented in communication technology.
In communication technology you want to save computing power and you want to save bandwidth.
If you want to communicate that to someone else,
you just need to reduce the amount of information instead of the whole information.
With a reduced amount of information you can check that parts of the information is contained in that information.
So Merkle trees, maybe someone can look it up on where Merkle trees came from.
I believe they came from communication technologies.
They wanted to make communication more light.

---

### ChatGPT: "meaning"

What your professor is trying to get at is the **real reason why Bitcoin uses a Merkle tree instead of just hashing all transactions once**. It’s not just “because trees are cool”—it’s about **efficient verification**.

---

#### 🌳 Merkle trees in Bitcoin

![Image](https://images.openai.com/static-rsc-4/Fr_rTcvPICcucNBjBHMP-tfDaOaOEBF6SBILS9yXZ66cbBQB062NRuBi9t3xdBiFvam65SURTt-KuPztJ6O9c2I8DT-f7Tnw1gbEIW7P-6mW1-ipYYbDq4age4vW01od_FiUleEzyT1FNkLNqgD0wjtp7Ip-V0J59nHpzT_K6vf-re17vQgvuHIVEN0redZj?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/x22Wh4a7kXb7-j9RTMpH-CAUporq96wLL3v0hHso-FCI-XWvX6tRqR-7jzGxFgrOWb7lPBwWhBrqLuAkd0EsJDsG1baYXOmsh-QqyMhaCd4EHiQf7uXk6DitBhFMyHhrrE0F27wEB1ODwe9epVHT_PzReuzEQ2lVeeCTHmgC5vha-tD6pFSlFuNNqHEMYy-n?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/Zrpam3qw-R7C74N_A9tIvmgydTOtX8zYTE34E_6BiQ-ozU6YeGmmDtisMzYduyRR0w_YfdBqLrvxlYntPDoqbkyimFuM9Jw3zQysF-DEh-oGfWs_reRErKb_Taxwseu_94PDWju2wy8LyCdo8uQQtZGjZ47gcyXMKZnhMXTRAip2uNJcl4W9fc98SpO5XdT5?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/z8HUwZLMSyWyoN8BzqogYNfmQO_IznjcqD3VoJggRFr06h3nz1PMUoNBDSawcpb_cV2xZgha0DMvEo64TlTu25ccZNw59dZ7yhBaBWA7M9QgE674ppsQwRA_jFHI8b3G39A8_wwI2Ttb7PuAOmCGQYE9Nox93fwFK3byD4TnQdB3Zx6A0722-Jsdhgimd5SG?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/DH6rB4xJxEHs5i0RRh_7hb8q9FX6MbjEIxXKYBZZlNo3hribfFEqR9Gp9qyFok6qC06gaJPxRXe2X8FvV02iYHQ_GBYcWryK7nAvC1eAFUufSoX_8AxeQnnAqMf4eupgTizu3yEGWflVShPSlGT_d65fpT7yyGQCspxex3m4RFON2kD4Z1vNJ8kEfiFT6vhY?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/EPTmPUe64XfvcziNblsK9F9smItrU6MPdYBywAdXpNxCc_WezX0-TtY4qMVyX7EYtT5kQ8eofNdbKrJHsJN4MyXEv8WTK-uq9f5biCtsQt3HaiUQdoOCPwS1_fTIIEm29-yNAYmp9eLHo5vNh4BAbJ4PCP0ywFXDqYeeNvUTiCK9zuKpwpzLyghnZXxZXP_G?purpose=fullsize)

---

#### 🧠 The core problem

You have:

* A block with **many transactions**
* Only **one hash stored in the block header** → the *Merkle root*

Now someone tells you:

> “Transaction X is inside this block.”

👉 How do you verify that?

---

#### ❌ If we just hashed everything once

Imagine we did:

* Concatenate all transactions
* Compute one big hash

Then:

👉 To verify **one transaction**, you would need:

* **ALL transactions in the block**

Because:

* You must recompute the full hash
* You can’t isolate just one part

---

#### ✅ What the Merkle tree solves

With a Merkle tree:

👉 You only need a **small subset of data**, called a *Merkle proof*

---

#### 🔍 Example (your lecture)

Say we have 4 transactions:

* Tx0, Tx1, Tx2, Tx3

To prove that **Tx1 is in the block**, you only need:

* Tx1 itself
* The hash of its sibling (Tx0 or its hash)
* The hash of the other subtree (Tx2–Tx3 combined)

👉 Not all 4 transactions—just a few hashes.

---

#### 📉 Why this is powerful

Instead of needing:

* **n transactions**

You only need:

* **log₂(n) hashes**

So if a block has:

* 1,000 transactions → you need ~10 hashes
* 1,000,000 transactions → you need ~20 hashes

👉 Massive reduction in data.

---

#### ⚡ Why this matters

##### 1. 🪶 Lightweight verification (SPV nodes)

“Light nodes” don’t store the full blockchain.

They can:

* Verify their transaction is included
* Without downloading everything

👉 This is crucial for scalability.

---

##### 2. 🌐 Efficient communication

When nodes talk to each other:

* They don’t send all transactions
* Only the **relevant proof path**

👉 Saves bandwidth.

---

##### 3. 🔍 Localizing changes

If one transaction changes:

* Only its path up the tree changes
* Not the entire structure

👉 Easier to detect *where* something changed.

---

#### 🧩 What the professor’s answer means

The key takeaway:

> With a simple hash → you need **everything** to verify anything
> With a Merkle tree → you need **very little** to verify something

That’s the “aha” moment.

---

#### 🧠 Simple analogy

Think of:

* A Merkle tree like a **family tree of hashes**

To prove someone belongs:

* You don’t need the entire population
* Just their **ancestors up to the root**

---

#### 🏁 Bottom line

Merkle trees are used because they allow:

👉 **Efficient, partial verification of data**

* Minimal data needed
* Minimal computation
* Works even for lightweight clients

---

If you want, I can walk you through a **step-by-step Merkle proof with real numbers**—that usually makes this concept click instantly.

---

### ChatGPT: "where is the merkle tree stored"

Short answer: **the full Merkle tree is *not* stored anywhere on the blockchain**—only its **root hash** is.

---

#### 📦 Where the Merkle root lives (on-chain)

![Image](https://images.openai.com/static-rsc-4/2fYGJr4OKqLB38bOVj-6LLrLw1q2SFEzQXRyQD0gmCxG1RERT8VP_NC-rD-deP8FK8uzrVUzTaBCOwirxpYvQOnkJqWqyw7V4UMBf5ez4BFXVcSha3aP9zQqALqAXihkkfIKncTBP89PNyglMFxhCpNjyBqVw3i4pm3duf4REbLCKtF9MvmWl7Q9sqbSbuxp?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/DDtV_DO3cymIwC3dD_q74mOmc5XpWq4v52HoiAe6Ph8LKk_gFpv4iUfWkGSJ_BIkVrC4-5yRKiiNcuVzFVss3dlH0WSfgYlmrdaKmnDoCaPoWFRtHYAnnnhDyTqc1OUf04bkqKVACW4MFXDi-xIdP3liFZEMv9hBwApQ7FDBn_lQdAg-6gcvEhbGwKtnZIoR?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/wJ79ChSYKZ0If3D6f5kgFjkk1wVMr4G-QfAW_HesQDx46BliqMBoXHdbnA9ipTgDIDVus5E7oC4a2D6Ii-Ivo0sNnhbCsGjfwQay51LocnTAZqn9-zJlm5zmJBWe7PTtw_EjgxG6wjy1Muve8sBrLL-hhUfLabO0Du_rxWE1dHVTKQ-SBsABRv8OnGTKx_2g?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/smmwhVWdcm-vLgGU7P3DzY57TdeNsPD7Ef_-jpBHDDS_LL22mGSVtusVpFBg2JrVI-9QrBNiUg0N2pZ4p2WGKdbVYd4txE7-JysqVjU-CeZKw7zm0_CH4lZ9wyRQKeJl5_xFiGC7d9Lfg0CFomOZdrp-Vh2_TqwsFLAZarodJYtRKnTX6bK38ZcxGCZBZwna?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/EPTmPUe64XfvcziNblsK9F9smItrU6MPdYBywAdXpNxCc_WezX0-TtY4qMVyX7EYtT5kQ8eofNdbKrJHsJN4MyXEv8WTK-uq9f5biCtsQt3HaiUQdoOCPwS1_fTIIEm29-yNAYmp9eLHo5vNh4BAbJ4PCP0ywFXDqYeeNvUTiCK9zuKpwpzLyghnZXxZXP_G?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/bNVdhGeB7TC9Nb8EH4piw7t8GV-9roIAg21-RYL_OpJ4FUk6pnIMYucmKvu5lbJgGwXzbAmum701Ki667SWJL-itj1VqZSi8YKsz61G4M-ViXRotwYyEwKceo1ZCUG8Qw-hn0TDupFsryxPZxu7DqYdNarD0hjoRqIPsHZdTEYawMF8nlq2CMpvGNaUeSuLc?purpose=fullsize)

In Bitcoin:

* Each block has a **block header**
* The header contains a field called the **Merkle root (Tx_Root)**

👉 That single hash represents **all transactions in the block**

---

#### 📂 Where the actual transactions are

* The **full transactions** are stored in the **block body**
* Every full node keeps them (at least until pruning)

👉 From these transactions, any node can **recompute the Merkle tree when needed**

---

#### 🌳 So where is the Merkle tree itself?

👉 It is **not stored explicitly**

Instead:

* Nodes **build it on the fly** from the transactions
* Or cache parts of it temporarily

So:

* Stored permanently: ✅ transactions + Merkle root
* Not stored: ❌ full tree structure (all intermediate hashes)

---

#### 🤔 Why not store the whole tree?

Because it would be redundant:

* The tree can always be reconstructed from transactions
* Storing it would waste space
* Blockchain storage is expensive and should be minimal

👉 Only the **root** is needed for integrity

---

#### 🔍 What about Merkle proofs?

When someone wants to prove:

> “Transaction X is in this block”

They provide:

* The transaction
* The **necessary sibling hashes** (Merkle proof)

👉 These hashes are:

* Either computed by a full node
* Or retrieved from cached data

---

#### 🧠 Big picture

Think of it like this:

* **Block header** → stores the *fingerprint* (Merkle root)
* **Block body** → stores the *full data* (transactions)
* **Merkle tree** → reconstructed *when needed*

---

#### 🏁 One-line takeaway

👉 The blockchain stores **only the Merkle root**, not the full tree—
because the tree can always be rebuilt from the transactions.

---

If you want, I can show you **exactly how a node reconstructs a Merkle proof step-by-step**—that’s where this becomes very concrete.

---

### ChatGPT: "where is the block body stored"

In Bitcoin, the **block body is stored on every full node’s local copy of the blockchain**.

---

#### 💾 Where the block body actually lives

![Image](https://images.openai.com/static-rsc-4/UTv22I13tZuSthRsJLjWXGM96PPTG-H_eFAYOwI3tGTO6-jbEMYiMXttQSwXrKS_Q2jlWnjV0uwVpKIXz1Qyds8Op3COucUNWWbicxEz7Mr-G-35UsZQIvlHkfp1n7bEvUIJBvl7RzV718_gp12RG2RMNwig-wS2NAnRQFkx13VJvrzeUIbUrS0rDp44rhxQ?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/oRV9_pnfwwUTwjavSoB_g78Z8kuFnBv_qI4352P9Z2dagqRwjAejr5nlninOmA6fXXSV97usWsk934opc5DYX4J-T489FD9OZ7upspfuLTX9WeqbblflvNQ2t6egbbjK9pPYRQZRS2qlUf3OnoA9AzL60rdfIz96dzR9ch5DsE5XKKq4cjXe2WqKURowkr-H?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/0oOHLGVURH8hjSrSq-7b3SY0k_W5w-hl9sHqeMLcXkh647Mrjfpi4ujCAJ75hF4PopotpsrvztUYuGDYynYYhs4AYqSFuSs_cVAEleC4_ctNYWdqCpcjOlbrUw5RDnHWLJo4aLe2jMrYCbZm7i1Gs5IWg889X9ZscM_n8rxgCyxlGbvFuu59ubaG1ceWfh9d?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/wJ79ChSYKZ0If3D6f5kgFjkk1wVMr4G-QfAW_HesQDx46BliqMBoXHdbnA9ipTgDIDVus5E7oC4a2D6Ii-Ivo0sNnhbCsGjfwQay51LocnTAZqn9-zJlm5zmJBWe7PTtw_EjgxG6wjy1Muve8sBrLL-hhUfLabO0Du_rxWE1dHVTKQ-SBsABRv8OnGTKx_2g?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/g-g6iR1nYgjuWd9EZkIhSfpP0t-Af4FJMPS9LyRnFpwa3Zs_-Uz1agk_KOqf5LbumlR7L4-ppdrXwWGkGijSLNLuUITinex3e-e6O7tcdsyU1p54Gn8yl06FOnS-NhJrdFDPeHEKxf2BR68ezI8d851lk7iF94lVdjiUJrwl207diqWPHvekxEcnxKlftb2U?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/6VTdShzEOVSH_JFf6VI882tNN8kVNIIZtRmf3SQxQmEkp5jLpnq67cRMbDEhGtLDb4PhoiO74H-30Zpr7-_R1_zEOZsvcTi0LKjYQy38nHEHjo-dyCHdysUzG8FpRdqVTqz71xCh5r_ftV63XMIvt3ASYhWZdhZZm3HY7dVL_Ru7HmAJC_U8ZahE9PEscRMB?purpose=fullsize)

A **block** has two parts:

* **Header** → small (contains Merkle root, previous hash, etc.)
* **Body** → large (contains all transactions)

👉 The **block body (transactions)** is stored:

* On disk
* By **full nodes** in the network

---

#### 🌐 Distributed storage (important idea)

There is **no central place** where the block body is stored.

Instead:

* Thousands of nodes around the world each store a copy
* This is what makes Bitcoin decentralized

👉 The blockchain = **replicated database across many nodes**

---

#### 🖥️ On a technical level

If you run a full node (e.g., Bitcoin Core):

* Block bodies are stored in files like:

  * `blk00000.dat`, `blk00001.dat`, …
* These files contain:

  * Raw serialized blocks (including all transactions)

There’s also:

* A separate **index/database** (chainstate) for quick lookup

---

#### ⚖️ Different node types

##### 🧱 Full nodes

* Store **entire block bodies**
* Can fully verify everything

##### ✂️ Pruned nodes

* Keep only **recent block bodies**
* Delete older ones to save space
* Still verify everything before deleting

##### 🪶 Light nodes (SPV)

* Do **not store block bodies**
* Only store headers
* Request data when needed

---

#### 🔁 How it connects to Merkle trees

* Block body = all transactions
* From those → node builds the **Merkle tree**
* Stores only:

  * Transactions (body)
  * Merkle root (header)

👉 Tree itself is reconstructed when needed

---

#### 🧠 Big picture

* Block header → tiny, stored everywhere (even light nodes)
* Block body → large, stored by full nodes
* Network → distributes storage across participants

---

#### 🏁 One-line answer

👉 The block body is stored **locally on full nodes as part of their blockchain copy**, not in a single central place.

---

If you want, I can show you **how big the blockchain is today and how storage grows over time**—that gives a nice sense of scale.

---

### ChatGPT: "so the full Merkle tree is not stored anywhere on the blockchain, but the transactions are?"

Yes—that’s exactly right, with one small nuance.

---

#### ✅ What *is* stored in Bitcoin

##### 📦 Block body → stored

* All **transactions** in a block are stored
* Full nodes keep them on disk (unless pruning)

👉 This is the actual data of the blockchain

---

##### 🔗 Block header → stored

* Includes the **Merkle root** (a single hash summarizing all transactions)

👉 This is the compact “fingerprint” of the block’s contents

---

#### ❌ What is *not* stored

##### 🌳 Full Merkle tree → not stored

* Intermediate hashes (all the internal tree nodes) are **not permanently saved**
* They are:

  * Recomputed when needed, or
  * Temporarily cached

---

#### 🧠 Why this design makes sense

##### 1. 🧮 No information loss

* The Merkle tree can always be rebuilt from the transactions
* So storing it would be redundant

---

##### 2. 💾 Saves space

* Storing all intermediate hashes would increase blockchain size
* Bitcoin minimizes on-chain data

---

##### 3. ⚡ Still efficient when needed

* Nodes can quickly reconstruct:

  * The full Merkle tree
  * Or just the parts needed for a proof

---

#### 🔍 One subtle nuance

While the **blockchain itself doesn’t store the full tree**, some nodes may:

* Cache parts of it in memory
* Store indexes for faster access

👉 But that’s an **implementation detail**, not part of the protocol

---

#### 🏁 Clean mental model

* Transactions → ✅ stored
* Merkle root → ✅ stored
* Full Merkle tree → ❌ reconstructed on demand

---

#### 🧩 One-line takeaway

👉 The blockchain stores the **data (transactions)** and a **commitment (Merkle root)**—but not the full tree structure.

---

If you want, I can show you how a **Merkle proof is constructed step-by-step from stored transactions**—that’s where this becomes very concrete.

---

### ChatGPT: "what is a node in the real world? a computer?"

Yes—**in the real world, a “node” in Bitcoin is simply a computer (or device) running Bitcoin software**.

But that simple definition hides a few important nuances.

---

#### 💻 What a node actually is

![Image](https://images.openai.com/static-rsc-4/-_m1gwjkw0fUmKhyoDNTPZvWEw85VcLLvfmLp_PRnKtzglecI-DJTBOuJKeKWKV6KmKpiL883xdHM6c-f0m2WK9T66rqv-ER1XtHje6Rt96tQm30BJSgMmbgw3FJEBFJnKqTrdUWEfWwNE542gzPJkQ41Tl6M4W_gnyvWKQldwILSbHkaR5aEhrOUFKn7U6Y?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/TOPaVyrisv3Nm34sP2fEzSzqzxaG-CsVSTlp7ngoaEitSld30JrPlhEq2SvqXr81pwih0hilfJUmmd2Z9hV2B0v3UBd3r92SG8bPQRuHRLfihFQwp59QAhIHvSErA1FmH578zf0D_aIlG-uSqR9f7ycxs0Qccm8uK0TfzRzI1ZztYFRF0jPwWByoorQWNvuZ?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/t35QR6-0YbkByjc_dL8F73P1uUj1NwqhqyLzGNq3Lj99pB06lZDBiNqWG8V0r6cgaTpA_VRYXxXN_z_V55ri2sTAAqiBbThQRLMlr4nLORYdgKq4iHGsBNaNgIePcnkvY2Y-TJIai5mYpx2YdTPWVoBrw-mM5b3iVQLXeo7Yme6Lu9owZ1U8tKsL4_WTjPAC?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/gwrD0W4dzk6CnQU47QigrOgHS5nrXIVTFpg6PSARfiWHsRVtL8NYSkIK-UA_nD8y_8VmlWrKe_s14YVBz-4gY-PVlw9clb5VsLpD4Sx4dc3vbyEN3Bo0JsPs-kczqixcMGtCT-DiFGH6oZtCsPpqF57n_dxf0tr2w9jqnOg2V5teEjoV9M72X4LxoxWvm8qS?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/dD5FTKn64pbWnvelil8uhpHQbaEEBCgjrIKb4ZbQdgelNyE1B87mZYu-64SL3yIiiTubVRt66oajt3ifhZYQmgG93iiNj2GpP7YQdVYy8ZJdrj61wwz9gVIvNmYEOBYOut9Hf5t3fOW7uGOXetBPORTxcF7jmuWxbnddkRQxlTupkjvyfEZEATEQlJKQxL2v?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/i3AubBBZxN7__LnjSYLiFl5CITVLKqMJ-KVgsRyhSrDVcXW_qgNL8m2tZ28Yt9fXysA5_84IOOmuVBrKXbPfHCRA1zFyTqnZLnOBVFCcPmZ1onHyfe2N10E8HBe3VZHGYOtm33lrpO8F4N3P3wV0iayP9cwzXF9PRFyM_ekyuBARkw_tm1fo9w_EJsC_8KBu?purpose=fullsize)

A **node = hardware + software + network connection**

* **Hardware**: laptop, desktop, server, or even a Raspberry Pi
* **Software**: e.g. Bitcoin Core
* **Network**: connected to other nodes via the internet

👉 So yes, it’s a computer—but specifically one participating in the Bitcoin network.

---

#### 🌐 What nodes actually do

Depending on the type, a node can:

* Store blockchain data (blocks + transactions)
* Verify rules (no cheating, valid signatures, correct rewards)
* Relay transactions and blocks to other nodes
* Maintain a copy of the ledger

👉 Nodes are the “judges” of the system

---

#### ⚖️ Different kinds of nodes

##### 🧱 Full node

* Stores the full blockchain (or pruned version)
* Independently verifies everything
* Most important for decentralization

---

##### ⛏️ Mining node

* A full node + mining hardware
* Tries to create new blocks

---

##### 🪶 Light node (SPV)

* Runs on phones or lightweight apps
* Doesn’t store full data
* Asks full nodes for information

---

#### 🧠 Key insight

👉 A node is **not a special machine owned by Bitcoin**

There is no central server.

Instead:

* Thousands of independent computers
* All over the world
* Run the protocol voluntarily

---

#### 🏁 One-line answer

👉 A Bitcoin node is **any computer (or device) running the Bitcoin protocol and connected to the network**.

---

If you want, I can show you **what it takes to run your own node (hardware, storage, time, cost)**—it’s more accessible than most people think.

---

### ChatGPT: "does a full node store all transactions that have ever been on the blockchain"

Short answer: **yes—with one important exception.**

---

#### 🧱 Full nodes in Bitcoin

A **full node (non-pruned)** stores:

👉 **All blocks → which contain all transactions → from the genesis block to today**

So:

* Every confirmed transaction ever included in a block
* Is stored locally on that node

---

#### ⚠️ The exception: pruned full nodes

There are two types of full nodes:

##### 📦 Archival full node

* Stores **entire blockchain history**
* Keeps all transactions forever

👉 This is the “complete” version

---

##### ✂️ Pruned full node

* Verifies everything **first**
* Then deletes older block data to save space

👉 It keeps:

* Recent blocks
* The current state (UTXO set)

But:

* ❌ Does **not** keep all historical transactions

---

#### 🔍 Important nuance: mempool vs blockchain

* **Confirmed transactions** → stored in blocks
* **Unconfirmed transactions** → only in the mempool (temporary)

👉 Full nodes do **not** permanently store:

* Dropped or never-confirmed transactions

---

#### 🧠 Why pruning works

Even if a node deletes old transactions:

* It has already verified them
* It keeps the **UTXO set** (who owns what now)

👉 That’s enough to validate future transactions

---

#### 📊 Scale (rough intuition)

* Full blockchain (2026): hundreds of GB
* Pruned node: can run with just a few GB

👉 This makes running a node more accessible

---

#### 🏁 One-line takeaway

👉 A full node *can* store all historical transactions—but only **archival nodes actually keep everything**, while **pruned nodes discard old data after verification**.

---

If you want, I can explain **what exactly the UTXO set is and why it’s enough without full history**—that’s a key concept.

---

### ChatGPT: "so a pruned node cannot validate all PAST transactions" (see next prompt for clarification)

Not quite—the statement needs a small correction.

👉 A **pruned full node *can* validate all past transactions**—it just **doesn’t keep them afterward**.

---

#### 🧱 How pruned nodes work in Bitcoin

##### 🔄 Initial sync (important!)

When a pruned node starts from scratch:

1. It **downloads all blocks from genesis**
2. It **verifies every transaction and block**
3. It builds the **UTXO set** (current balances)

👉 So it *does* validate the entire history.

---

##### ✂️ After validation

Once old blocks are no longer needed:

* It **deletes old block data**
* Keeps only:

  * Recent blocks
  * The **UTXO set** (current spendable outputs)

---

#### 🧠 Why it still works

To validate *new* transactions, you don’t need full history.

You only need:

👉 The **UTXO set**
(which tells you what coins currently exist and who can spend them)

---

#### ⚠️ What a pruned node *cannot* do

After pruning, it cannot:

* Re-verify old transactions **from scratch again**
* Serve full historical blockchain data to others
* Answer queries like:

  * “Show me transaction from 2013”

👉 Because that data is gone locally

---

#### ✅ What it *can* still do

* Fully enforce Bitcoin rules
* Validate all new blocks and transactions
* Stay in consensus with the network

👉 It’s still a **fully validating node**

---

#### 🧩 Clean mental model

* **Before pruning** → verifies everything
* **After pruning** → forgets old data but keeps the result

Like:

> Doing all the math homework, then keeping only the final answers.

---

#### 🏁 One-line takeaway

👉 A pruned node **does validate the entire history once**, but **doesn’t store it afterward**, so it can’t re-check old data without re-downloading it.

---

If you want, I can explain **why the UTXO set is enough to secure Bitcoin without full history**—that’s a really elegant part of the design.

---

### ChatGPT: "so a pruned node cannot validate all past transactions AFTER PRUNING"

Yes—**that statement is correct, with one precise nuance.**

---

#### 🧱 In Bitcoin terms

👉 A **pruned node cannot re-validate all past transactions *after pruning***
because it no longer has the full historical data.

---

#### 🔄 Before vs after pruning

##### ✅ Before pruning (initial sync)

* The node downloads **all blocks**
* Verifies **every transaction from genesis**

👉 Full validation happens here

---

##### ✂️ After pruning

* Old blocks (and their transactions) are deleted
* Only kept:

  * Recent blocks
  * The **UTXO set**

👉 Now:

* ❌ Cannot re-check old transactions from scratch
* ❌ Cannot reconstruct full history locally

---

#### 🧠 Why it’s still safe

Even after pruning, the node can still:

* Validate **all new transactions**
* Enforce all consensus rules

Because it has:

👉 The **UTXO set** (current valid coins)

---

#### 🔍 What exactly is lost?

After pruning, the node cannot:

* Verify a claim like:

  > “This transaction from 2012 is valid”

…unless it **re-downloads the old blocks**

---

#### 🧩 Simple analogy

* Before pruning: you have the **entire accounting history**
* After pruning: you only keep the **current balances**

👉 You trust your balances because you verified everything earlier
—but you no longer have the old receipts

---

#### 🏁 Final takeaway

👉 A pruned node:

* **Has validated the entire history once**
* But **cannot validate past transactions again later without re-downloading them**

---

If you want, I can explain a subtle follow-up:
👉 *Why Bitcoin security does not require everyone to store full history*—that’s a really interesting design insight.

---

### ChatGPT: "difference between light node and pruned node"

Good question—these two are often confused because both use less storage, but they’re fundamentally different in **what they trust and verify**.

---

#### 🧱 Pruned node vs 🪶 light node in Bitcoin

![Image](https://images.openai.com/static-rsc-4/OpyZ0YrF6_HIQCMmUj_DJIZuzbrWCQX9mEAN_UGunFHZREFOz90wWcCw3w7AtfN0hgdpAz6r2bUCpTWofsvb07w1dd_gJ005y6SA7ykgymanFEEgATzennD2gGuRqHeYApBt4tlLzXcpYf-rZffugZe2joqk4iOEJkaQLw3JzGdQ1BLJwLSrDI3nBn2LauhS?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/IwlLLtVe9lPB7xDUMouLHqe22HI_pWV1yoaixMpb5vY_AGhF7XwjD4scrlpsvHBg6IzAKkiwF8JVxDU2W6geJzfo7Uq0brPx_VFNZO2deJR2tfLYd573B2exsjN_FUS336tHS0OfKRTRqMt8Q3TdJ1GuaTPltY_eR7hAKuAGu7sxF9jMRzJvKP_JWOKwJdXe?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/vdAFw5C_2ZRaiz2VH0Y7KP1b47PcIouPC6WYafXp0Z0GT7pQ9lSOrXfxlyMa5Ml5FlGT-4XkeCG_7DpY2D4dtOGXZJBCbqQNaEIw1bzLFpseBHRCCa1OsHLmW3adP0xIuYCl05m0LIEUSqO-pcbjmC6s7ohmOphvRDQK3mi9dCDAAg7dxZIbiOJDVtFCFjdH?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/gWtZfiO6u9IV5dChXeV4EpE8RwkvGupziPmQjrab5DjrSM3VgTcXgS5HB6AH9nSL9yam7_NFLMR-VLD3vIuq9am3lwJQ4ezOZ7mjZatrCPhSyQkjFKhHvycsAj5CVHenEzmflRhbQj3c6xq9ixGz98SteaBfHLvUMgov088OIMEsXb-eaSSfVhKBwhZg4Kmi?purpose=fullsize)

![Image](https://images.openai.com/static-rsc-4/CxLQlHRNbxR65NYtZiyrNvvxSPuB9RDIxD6mnLRIvNMrj-VyOBciYaMrViqe8HOarz6LzQir3Kv9NlrLTCsHmf6hxiIpH4vaZi-W0sm4xjv-FO7PcFdtaUlSVY9R0YkVx_1BJicWFSeB4fMyEO91-xFqPJ9L5VrJWDI_YFlX1ma6lPYOrvXkUa641LHK7zCf?purpose=fullsize)

---

#### 🧱 Pruned node (still a full node)

👉 A pruned node is a **full node that deletes old data after verifying it**

##### ✔️ What it does

* Downloads **all blocks from genesis**
* Verifies **every transaction and rule**
* Builds the **UTXO set**

##### ✂️ Then

* Deletes old block data to save space

##### 🔒 Trust model

* **Trusts no one**
* Fully independent verification

##### ⚠️ Limitation

* Cannot re-check old transactions later (data deleted)

---

#### 🪶 Light node (SPV node)

👉 A light node is a **simplified client that does NOT fully verify the blockchain**

##### ✔️ What it does

* Downloads only **block headers** (very small)
* Requests data (e.g. transactions, proofs) from full nodes

##### 🔍 Verification method

* Uses **Merkle proofs** to check inclusion of transactions

##### 🔒 Trust model

* **Partially trusts full nodes**
* Assumes the longest chain is valid

---

#### ⚖️ Key differences (the essence)

##### 🧠 Verification

* Pruned node → **full verification**
* Light node → **partial verification**

---

##### 💾 Data storage

* Pruned node → small (but once had full data)
* Light node → very small (never had full data)

---

##### 🤝 Trust

* Pruned node → **trustless**
* Light node → **trust-minimized, but not trustless**

---

##### ⚡ Resource usage

* Pruned node → moderate (CPU + initial sync heavy)
* Light node → very low (good for phones)

---

#### 🧩 Simple analogy

* **Pruned node** =
  “I read the entire book, verified it, then kept only the summary.”

* **Light node** =
  “I didn’t read the book—I ask others and check short proofs.”

---

#### 🏁 One-line takeaway

👉 A **pruned node is still a fully validating node**, while a **light node relies on others and only verifies partially**.

---

If you want, I can explain **exactly how a light node can still be reasonably secure despite not verifying everything**—that’s where Merkle proofs really shine.

---

## org

Okay, good.
With this we can stop today.
What we do next time is then to discuss what happens if we have more than one transaction.
How do we actually send if we have one bitcoin?
But I only want to pay you a half bitcoin.
How can I cut a bitcoin into two pieces?
I don't know.
I know that, you don't know that.
Next time we discuss that.
Some other very interesting issues.
Then we also go to some other kind of blockchain principles.
Because at the moment **bitcoin is the only blockchain that is still using proof of work**.
But there are others which use **proof of stake**.
Then we discuss how proof of stake works and what they actually do.

I will think if I can give you already some kind of exercises,
maybe it will just be some questions that you can ask for yourself,
so that you can recap that you understood all the things.

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
