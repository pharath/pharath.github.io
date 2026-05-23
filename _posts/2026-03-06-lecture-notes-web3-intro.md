---
title: "Web3 and Distributed Ledger Technology - Introduction"
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

## Is Blockchain a Protocol?

Yes, a blockchain is fundamentally a **protocol** (or a set of protocols). It functions as a foundational set of rules that dictate how decentralized computers (nodes) must communicate, verify transactions, achieve consensus, and securely record data on a distributed ledger.

### Why is it a protocol?

Just as HTTP tells web servers and browsers how to communicate across the internet, a blockchain protocol dictates the exact rules of a decentralized network. A blockchain protocol establishes:

* **Data Structure**: Defines how transaction data is packaged into "blocks" and securely chained together.
* **Consensus Mechanisms**: Dictates the rules for how nodes agree on the validity of transactions without needing a central authority (e.g., Proof of Work or Proof of Stake).
* **Network Communication**: Standardizes how computers discover each other, share data, and maintain identical copies of the ledger.

### Examples of Blockchain Protocols

While "blockchain" is the underlying concept, specific networks use their own unique protocols. Some of the most widely known include:

* **Bitcoin**: A protocol designed specifically for peer-to-peer electronic cash transfers using a Proof-of-Work (PoW) consensus model.
* **Ethereum**: A programmable protocol that allows developers to build decentralized applications (dApps) and automated Smart Contracts.
* **Hyperledger Fabric**: An enterprise-focused protocol designed for private, permissioned business networks rather than public cryptocurrency trading.

## Fiat vs. Crypto

- digital vs. non-digital
- tangible (cash) vs. non-tangible (only digital money)
- centralized vs. decentralized
- who is backing the money?
  - is every dollar backed by gold?
    - it used to be like that with the gold standard in the US where you could not print dollar when it was not backed by gold
    - but now a dollar is just "paper", ie. it is just "trust" which makes Fiat and Crypto in a way similar again
      - we all have to trust that for 10€ we get something that has the value of 10€
  - is every cryptotoken backed by anything?
    - there is the "trust" that we get something out of crypto, but there is actually no backing behind that
    - ~~it is just the "trust" in the local government that they keep care of the money and that we get some money~~
    - what is the "trust" in the case of crypto? Trust in algorithms? Trust in decentralization? Trust in some kind of nice servers and the internet?
      - in the end you are, again, "trusting" that if you have 1 BTC token you can exchange the token for 100,000€ (or whatever the current exchange rate is)
  - cryptocurrency is no real currency/money!
    - because a currency is something that is being issued by a bank (eg. Bundesbank, state bank, European Bank, Federal Bank, etc.)
    - it is not really being understood as some kind of currency/money, but as some kind of Cryptotokens with which people associate a value

## Example for Trust: Rai Stones of the Yap Islands in Micronesia

- People said: "We acknowledge the ownership of something or the right to do something with one of these stones."
  - the right to work on this particular kind of land and to harvest the fruits that come from that particular kind of land is being associated with that stone
  - they did not do carving into the stone, it was the public knowledge, the public history, the common sense that he owns a particular house - they did not have ownership of land - but he has the right to work on this particular kind of land on the eastern side of the island
- the interesting thing is that these stones were not available on this particular island, they had to get the stones from another island 600 km away (by ship/boat), so at least there is some kind of effort needed to get that, ie. it is not something that you can completely replicate
- and then sth interesting happened: on one of the trips to get a new stone the stone sank into the sea
  - on the one hand you would think "a million euro are being drowned" or "a 100 kg of gold is being drowned", what did the people from the island do?
    - when before they associated something with a stone that is on the northern part of the island, now they associate something with a stone that is somewhere down in the sea
    - "this only works as long as everybody adheres to the rules"
- but then something happened, it is part of the bad German history, the Germans that occupied the island wanted the people on the island to work for the German occupants, but the people on the island said "no, we are not doing that"
  - so the Germans marked all the stones with a white cross and said "that's our stone now, but you can get it back if you work for us" and then people started working for the occupants (eg. paving the streets, etc.)
  - and then they removed the white crosses and then the whole belief was still in the stones
  - this was at the beginning of the century, nowadays it is just a nice piece of history
  - the whole thing is that we believe in some kind of these rituals, we believe in the fact that this is associated with some assets and that we can exchange it for some other assets
  - it is an example that is often used in the area of Bitcoin or Blockchain
    - because in the next lecture we will talk about how to mine Bitcoins and how to build Bitcoins and what the Proof of Work, etc. is and it takes time and **it takes work** and that is similar to the Rai Stones, ie. in order to get such a stone you need to go to another island, you have to somehow prepare the stone and you have to bring it back (because it is something that you cannot get from this particular island, it is something from the outside and that makes it so special)
- and there is another thing: Milton Friedman who won a Nobel Price in Economics made another example:
  - this happened during the gold standard: during the gold standard it happened that the US said that they are no longer backing each dollar with some gold which was the case at the beginning, ie. for every dollar there was an appropriate amount of gold somewhere else, but then they said that this is no longer the case and that they can just print dollars now
  - but then the French got suspicious because a lot of gold from France was in the New York bank and then they said that they would like to have their gold back, but it did not make sense to take it over from New York to France physically, so they said they would like to have their gold now somehow separated from the gold of the US (which was the case, they just had a lot of gold in a big room)
  - so what they did is they took the French gold from one side to the other side (so, that is the gold of the US and that is the gold of the French)
  - and then suddenly the people in the US thought "what is happening here? The dollar is no longer backed. The French are already taking their gold. Is our dollar still reliable?" and that led to the first economic crisis at the beginning of this century and this was discussed by a lot of papers that were written by Milton Friedman who got a Noble Prize for observing this whole issue
    - this has a lot to do with the Crypto stuff

## slide: "What do you think of when you hear the term blockchain?"

- on this slide, these are things, when you talk about Bitcoin or Blockchain, this is what a lot of people have in mind:
- **dark net**
  - that was at the beginning, when we started discussing Blockchain about 7 years ago and when you were giving presentations on Blockchain and Bitcoin people were like "Ah, this is the thing you buy murder/drugs/weapons with on the dark net."
  - then it was just **"Bitcoin and Cryptocurrencies"** which is still the case, eg. when we discuss with industry or business people they always want to understand "What is this Bitcoin about? How is it being built? Why is it worth a 100,000 €", but they do not really see that we can have really **new business models** with this and there have been a lot of startups
    - just as a nice example for a **new business model** that suddenly comes up with Blockchain (because it has to do with alcohol): there was one brewery and this brewery did not have much success, but it was still too large to shut down, so what they did is, they issued some tokens, ie. NFTs (which we will see later on), and you could buy such a token and by buying this token you get beer for your life, that is the promise, ie. you buy a token like a voucher and for this voucher you get a 140 cans of beer each year (guaranteed for 3 years, if the business model survives)
      - if you buy 10 of these NFTs then you have the right to get your own beer, ie. a beer under your own brand (some colleagues of Prof. Prinz did that and he profited from it because then they had a 140 cans of beer, so he got some for free and then they brewed their own beer and called it "Web3 Beer" or similar which means that they suddenly became brewery owners/beer brewers because they got shares and these shares of this particular company are being expressed by a token that is sitting on a blockchain - we will see later on how this works)
- **energy consumption**
  - 3 or 4 years ago talking about blockchain meant "Blockchain is bad because of energy consumption! It needs too much energy.", even if you try to establish a sustainability program or if you want to do something "green" then people said "No, we cannot do that because blockchain is the problem. Blockchain has a high energy consumption.", but this is no longer the case and we will also learn about why this was the case and we will learn about Ethereum and what Ethereum did to overcome that, but there are some kind of second level chains and all these kinds of approaches that help us to overcome the whole energy issue
  - but there is another reason why nobody is talking about blockchain and energy any more: AI, on the one hand with the advent of AI everybody wanted to talk about AI
  - funny thing: people are saying "Blockchain is very interesting, I have heard so much about blockchain, but I never understood what it is." to Prof. Prinz, to which he replies "Did you ever understand how a LLM works?", but they are using it
    - this is an interesting observation because everybody wants to understand how this particular Bitcoin works and why it is so expensive and things like that and then they get in their car that has some kind of self navigation that they did not care about
  - so, energy consumption is no longer an issue because there is another enemy in town
- **banks and notaries**
  - The first promise was that we get rid of banks. We get rid of notaries. We don't need notaries anymore because we have a blockchain. Big, big mistake because it was completely over seven. I've been in running around there. There was some lady and she said, yeah, we get rid of banks. We get rid of notaries. We get rid of states. We don't need them anymore. We have blockchain that solves all problems of the world. 
    - Completely rubbish because it was complete overselling, which led to the problem that people, there was too high expectation. I think at the end of the whole lecture you will learn what blockchain can do, what it can't do. 
    - And in Germany, selling a new technology that you can get rid of notaries or something like that is, in Germany you can't do that because we believe in banks and notaries. If you go to another country where you probably have not such a trust in particular, public bodies, there you have much better success with that. But in Germany we somehow trust our government. So that's the problem.
- Okay, we talk about **keys and hashes**.
  - Yeah, that's cool. Yeah, a lot of keys and hashes in blockchain. Blockchain is just a big bunch of hashes.
- In the end, you can talk about the **DSGVO or GDPR**, which is also something.
  - You can't use the blockchain because it's not GDPR compatible. Not sure, but that's also something that we heard.

## When was your first time that you heard about blockchain?

- When was your first time that you heard about blockchain?
  - Yeah, I mean, we started blockchain, I guess I said 700 euros. It was 800 euros. And we didn't buy. At least not me, maybe some of my colleagues. But I have played around with 50 euros or 100 euros to see how to understand it. I didn't believe that it's increasing so much. No one could have believed it. Nobody could have. And it's always this thing, you could become a crypto millionaire. You could become a millionaire by buying Apple shares or Rheinmetall shares. It's always the same thing. Sometimes these things come up. 
  - So it was 17 with Bitcoin. Someone else got in earlier? Who knew about Bitcoin and blockchain before 17?
    - Oh, you got it. First one.
    - Yeah, actually, for me, it was that I heard about it just shortly after it came up. There were actually some articles about it in the Spiegel, the Spiegel, this German magazine. And then a friend of mine, with whom I started computer science, long long time ago in Bonn, he told me about it saying "I just set up a miner to get some Bitcoins. That's the latest shit.", so I also did it on the gaming PC of my son. And then you see something happening there and I completely had no understanding what it's about. Although I was already a professor of computer science. And so I said, well, what's it about? I mean, it ran for a day and then I stopped it. Unfortunately. So because at that time - and this is something we discuss next lecture - you could have become a millionaire if you would have known, but that's always the case.
    - Okay, so 17 is the first one and all the rest just heard about it later on then.
    - student: I think like I heard about it around like 2000, you know, just from like computer science. And also like just the mysterious creator and all that drama behind it. But how from the years was like, it was difficult to put any money online because of infrastructure issues. But yeah.
      - professor: yeah, that was the case also in the beginning, going to get a Bitcoin was not that easy. Yeah, I mean, all the hassle of getting funding in exchange and trusting them and so on.

## Web3

- Okay, so some other pictures here. And with this I would like to explain also reasons why Bitcoin and blockchain and Web3 at all. I mean, the term "Web3", is that known? Yeah? Do you understand this "Web3"?
  - student1: Yeah, Web3 is the possibility ... Now you put me on the spot. I would say Web3 is somewhat giving the users the option to the web for like not paying people but compensating everyone fairly in a personal way.
    - prof: Yeah, okay, it's some kind of monetization fair monetization of things. Yeah, right. That's perfect. Yeah.
  - student2: I think the biggest point I remember when I encountered this concept is that in web1, you just display content, you have the GET request, that's all you get.
    - prof: Okay, that's my story.
  - student2: But basically, you own your stuff. That's it.
  - prof: Okay, and sometimes people say, well, isn't that the semantic web? Wasn't Web 3.0 the semantic web? And that's a discussion I have with my colleague. He's in my department and he's doing a group of augmented reality and he's very much with Tim Berners Lee. You know Tim Berners Lee? Tim Berners Lee is the inventor of the world wide web. Extremely famous person, extremely important person, because he laid the foundations for all the things we are doing nowadays. And he also then said, we have a web 3.0 and web 3.0 is the semantic one which is that where you describe elements on the web, not in HTML, which has no real semantics instead of just, well, this is the header or that's just the paragraph or so. But really have semantics like well, this is the name, this is an address and things like that. So you describe content semantically and that's the web 3.0, which is very often mistaken as web3, which is completely different. Yeah, and we just edited a book on "web3 tokenizing the future" and you got the final proof print of the cover and it says "Web 3.0". And then, oh my God, what's happening there? But you could change it. Yeah, so, not getting into trouble with Tim Berners Lee, something like that. Such a famous person.

## slide: Digitalization (of a Bookkeeping Department)

- But, okay, so, but let's talk about digitalization and and I would like to get you some one of the stories we need to get into **WHY** we are talking
about blockchain and bitcoin in general.
- So this is a picture I took from a calendar. And it's somehow a bookkeeping department around the turn of the century. And you see, it's not digitized. No computers yet. But how would you digitize that? I mean, it's your task. What can you identify as elements that you can apply digitalization on? How would you translate that into a digital program?
  - student: First of all, you have to scan all the stuff.
  - prof: So, okay, scanning and where will you put it? In a database. That's the database. Okay. And that is (pointing finger at picture)? That's an application program that is doing something.
  - student: SQL?
  - prof: SQL yeah, maybe he (pointing finger at picture) is SQL. Because he's retrieving documents. He's the SQL guy.
- Okay, so in order to digitize that, our immediate reaction is this becomes a database and these are application programs that are working on the database. Fine. This is what you're educated for. This is something you can do immediately. Or ChatGPT does it for you. But that's actually what you can do.

## slide: Network (of Bookkeeping Departments) and the Cloud

- Next thing is, oh, there are more than this. That's a problem! And they have different approaches to organize the whole thing. We're doing business. We're doing networks. We have to exchange with other people. What's the problem now?
  - That in your database, you have a complete different state than in that database.
    - Because some information was lost. One of the programs didn't work correctly.
  - And soon we have all the discussions about: "I owe you something" and the rest says "No, you don't owe me that one because you didn't deliver 100 pieces, just 99 pieces." And you have all the litigation processes of negotiating that.
  - The problem really is that we have interfaces, sometimes they work, we have to develop new interfaces, sometimes information gets lost. And this information then leads to inconsistent states in the overall business community. We don't have a trusted single state. Everybody has its own state of the process. And you need to negotiate that we believe both in this particular state.
    - So what we somehow need is a solution that gives us the trust that when we say "this is worth something" or "this is 100 pieces" or "this is a measure of a particular machine or something" that we all have a consensus that this is true. We all agree on that. We have some kind agreement on a certain state in a business process. So what was the immediate solution for that?
      - student: standardization?
        - prof: We can standardize the interfaces, that's one thing. Costs you a lot of money. I mean, very often when you ask people that run some kind of business, not an IT business, they say, can I access your systems? I need an API for that. And they say "No, it's extremely expensive." They talk to the IT guys and they said developing the API and providing an interface is extremely expensive. It's one thing.
      - student: The intuitive solution would be to put everything in one place or for every request, access or libraries
        - prof: Everything in one place. Yeah, same solution.
      - student: Yeah, and the application also. Right? Decentralization comes to application.
        - prof: Yeah, application, but we are not yet in the decentralization. We are still in the centralization case.
      - prof: Putting everything in one place. **Cloud**. That's the promise of the cloud. The promise of the cloud is actually, yeah, give me all your data. I put it in one place and forget about any **interoperability issues**. I solve it for you. Because internally, we all have the same **standard**.
        - So that's why SAP is so successful. Everybody's using **SAP** because if everybody's using SAP, well, we don't have interoperability problems. We have a lot of other problems then, but - I'll tell you Fraunhofer moved to SAP since four years now. It's somehow working. It's something strange. But that's the promise.
        - On the other hand, with this particular kind of cloud stuff, yeah, we get into a complete different problem. And then we see if I have it here to show that. Because we are somehow being locked into the SAP or the whole cloud stuff. We are completely reliant on this particular thing. So for example, let me see. I can show you that one, for example. I just took this. It just was a personal example. **T-Online**. It's a German company, trusted. My father was a T-Online customer. He got older and older. And he wasn't any longer able really to work on this whole thing by himself. And so he moved out of his place and we just cancelled the T-Online contract. And some of you probably it was my fault, I didn't talk about the **email address**. Because we cancelled the T-Online contract and they said, "okay, you cancelled the T-Online contract. We are now moving you to a nonpayable contract.", but we had to answer that within three months. It's a long time. A long time. So we had to do something within three months. But since we had some trouble with (...). In the meantime, this was working fine. But we didn't look into the emails. So after three months, the email was gone. The email was gone. His whole **identity** was gone. His whole **identity** with this particular **central provider** was gone. Retrieving this particular email address was not possible. Because he recovered now, he wanted to get back to Facebook. He could not recover his Facebook account because his Facebook account was associated with this particular email address. And all the rest. And that led really, from my personal experience, really to the fact that we are really getting our whole identity from some **central identity providers**. And they are called **T-Online, Google, RWTH Aachen**, and this is also some kind of identity. And if you lose that identity, which is basically your **email address**, it's your identity to the digital world, you get lost. I mean, there's **no ways to recover it**. And that's when Selin also comes into the game when she talks about **digital identities that you own by yourself**, which is your own identity that you can provide.

## slide: "We rely on central services and platforms"

- Something else is about this one. And this is what I often like to show to people from companies. I mean, one year ago, you could show a Tesla. And when you were asking, who owns a Tesla, people were saying, "yeah, I own a Tesla". It was a trend. If you do it nowadays, everybody's shy saying "Yeah, I bought it a year ago."
- But something that is interesting is really that this kind of Tesla, it gets 120 software updates in two years. Cool. And that is what people thought, well, that is what Tesla really is good for. They know how to digitize a car. 120 updates per year and VW and BMW are not able to do, but they do the same thing. I just like to use this particular example, because there is a particular person who is responsible for all the updates. And that's him. And then you think by yourself, okay, this guy has the final power to send an update. And these stupid idiots in Germany should no longer drive a Tesla because they're just against politicians in the US or like that. So you completely get into the hands of this particular guys whether they are for good or for bad. And that's the whole thing that led really in the end, then, to the whole discussion on blockchain. Yeah.
- Yeah. And that's really the fact that people thought, well, for all the stuff where we need some kind of **forgery proof storage of transaction** and foremost, this is actually **money transfer**. We need a **central authority**. And I just took a picture here of Frankfurt, where we have all the banks, but we trust PayPal, we trust in our TÜV who's proven something, we trust Sparkasse where I have my bank account. We trust in them. And actually, they they work and we can trust in them. We can say that you kind of **trust** it.
- On the other hand, the whole trust in their processes is based on **organizational trust**, ie. we believe that they don't do anything wrong. But their IT is actually running on big databases. And if you ever programmed a database, the first thing when you install a database is that you have to provide an admin password. And that's the same for these people. Those people who have the admin password can change the database. Obviously, they maybe have four eye witnesses that you can only use that and it's all checked and so on. But it's a combination of some kind of technical measures and some kind of organizational measures. And you have to **trust** in this. And that was the point. And people thought, well, we should think about something different for that.

## slide: "It all started in 2008 with a white paper"

- And that led finally then to this particular paper in 2008, with the white paper. And just to make sure, I mean, 2008 is somehow difficult to remember, what was 2008? Yeah, I mean, that was the phone to have in 2008. It wasn't an iPhone. It was a blackberry. It was a keyboard and a small screen. And if you have a blackberry, you are a successful business person. The Olympic Games have been in Beijing. And that was the President of the United States. It's already quite some time ago. And at this time, this particular guy, women, group, whoever wrote this paper on **"Bitcoin: A Peer-to-Peer Electronic Cash System" by Satoshi Nakamoto** and this paper has been published. And also the interesting thing is that this paper has not been published at a journal and not at a conference, not at a blockchain conference because they did not exist yet, and not at another conference on distributed systems or whatever. Just on a distribution list. It was a white paper. Just being there. And the name, nobody knows who it is. There are some speculations who could have been. I think it probably was a group of people. But it's a nice mystery. At least, you can talk about five minutes in the lecture. But it's a nice mystery about the whole thing, what's going on there. So that is something that actually then happened with this paper, that was implemented at the beginning of 2009. So on the 2nd of January 2009, the first implementation of the system that has been described here was live and since then we have Bitcoins. So that laid out the foundation for something that really took off then.

## slide: "How can you manage an account balance between several partners without a central trusted authority?"

- But the whole question that actually has been discussed in this paper or that was discussed is really **"how can you manage an account balance between several partners without a central trust authority?"** That's actually the whole question that's behind blockchain. The question is, we want to handle an account balance, eg. money, but it could also be the fact that just, stupid stuff, but just the fact that this has about 20 keys. Or the fact that today it's about 17 degrees outside or the fact that it hasn't been raining for 2 months such that we get some insurance money back or so. How can we manage this particular fact between several partners without a central trust authority? Because until then, we always, always needed a trusted authority.
- student: I think like transparency that everyone can see at the same time.
  - prof: That's also something about seeing everything. But that comes to the next level. But that was the basic question.

## slides: "tally stick" (2 slides)

- And I said, well, **that was impossible beforehand** without having the banks or something like that. And even without digitalization, we have banks already ... we have ... or some other people who just had this kind of authority to do something ... registry or something like that.
- **But there was already something before** that allowed us to store in a decentralized way a particular fact or some kind of transactions without any digital medium and it was available already. Someone knows about this already?
  - student: Internet?
    - prof: No, before internet. Middle age.
  - student: A ledger?
    - prof: Yeah, a ledger is ... but a ledger ... do you know what a ledger ... you heard about distributed ledger, but a ledger normally was something like a book. But it's not decentralized. You always have it centralized somewhere.
  - student: I think it's like **"Kerbholz"**.
    - prof: Yeah, that's what I was looking for. **"Kerbholz"** is the fine thing. And the English word is **"tally stick"**. A tally stick is the medieval blockchain. And that's how it looks like. 
- And actually also learn about it from this particular block here. And then compare blockchain with a tally stick. And I think it's a brilliant example. What's a tally stick? A tally stick is something. So the whole idea is the following. We need to somehow make clear that you owe me 2 Taler. We don't have euros in the middle age. We have just Taler. 2 Taler. How can you do that?
  - He's a businessman coming from Düsseldorf. I'm coming from Cologne. So he's coming from Düsseldorf. I'm coming from Cologne. And you know there's a big rivalry between Cologne and Düsseldorf. We always make jokes about those cities. So you wouldn't trust it. So how can I trust in him that next time we see on the market in 1502 that we still have the case that he owes me 2 Taler.
    - So what I can do now is we can go to a **third party**. They put it on a **ledger**. But in opposition, you just put it on some kind of paper. They have paper. So we have one, two. See these are the two Taler that he owes me. Okay, but we have to do something with, we have to **decentralize** it. And what I do is now, I just cut it in a certain very strange way. Okay, so now we have two pieces. On two pieces, we have the two strokes (auf Papier gezeichnete Striche), which means 2 Taler. That's for him. And that's for me. And now he goes back to Düsseldorf. I go back to Cologne. Next year we meet at the market here in Aachen. They say, hey, come on, get me the two euros. I accept it. Now I'm cheating. And in the meantime, I couldn't stop thinking about them. Give me back my three Taler. And then he says, hold on. There are no 3 Taler. You have been cheating. Obviously, you have been cheating. And that's how this kind of tally stick actually works. And that's really a cool thing. Actually, when you look at this, this is a copy of Partner B and Copy Partner A. This is the shared data over here. So the two Taler and maybe some .... And that's the cryptographic - next lecture we'll learn about this - **cryptographic link** actually works between the two things. Because that doesn't really scale. I mean, we can do it for two people, three people, five people. It doesn't really scale.
      - student question: But what I say is, for example, you were in the pub, I went to the second part actually and I couldn't do anything about it. We just wrote something and it was all... 
        - prof: Okay, so yeah, you still could cheat me in this way. But this is still what you can also do with... Somehow would that neglect the whole validity of this kind of stuff? Yeah, you still can do that. Yeah, that's possible. But at least we cannot cheat about the certain amount. That's not possible.
- And also what is important here, and this is also something we learn about next time, is that we need to find **consensus**. We need to have an agreement. We need to have a consensus that he owes me 2 Taler. And by finding the consensus, we put it in our decentralized ledger, that's what we do. And since computer science then came up, Satoshi Nakamoto thought, well, we have enough technologies available to implement something like that now. There has been **digital cash** and things like that beforehand already. That was not new. I mean, there have been other approaches. But that was the **first time** that it was completely done in a **decentralized** way.

## slide: "Emergence of Web1" (Seite 17, 18)

- And that leads then actually to the emergence of this whole Web 1, 2 and 3. So, a colleague of mine, Lisa, she investigated into that, and in order to find some more reasons why the different things actually happened. And the first thing was actually the **Web 1**, **the Internet**.
  - I mean, there was a time before the Internet, for young people like you. I mean, when I started studying, my computer experience was an IBM 360 with punch cards at the University of Bonn, and at home I had an IBM PC and before that a Commodore. And the big cultural shock was that at home I was already programming Commodore PCs and IBM PCs with BASIC and stuff like that. And then I went to study computer science at the University of Bonn and they gave us punch cards.
- So that happened at this point in time, but all these things were not connected. They were not connected. There may have been some kind of communication lines between them, but it was not standardized.
- But then there was the cold war. And then there was the whole idea really to get from non-digital communication to digital communication. So, based on the whole issues of the **cold war**, that we need to have communication, rapid communication between different partners, easy way between electronic machines, there was the need to find some way of technology that allows us to do that. And that was at the beginning really the **ARPANET**. I mean, it really was the ARPANET that has been developed by the US based on a funding by DARPA (which is a part of the US Defense Department DoD). And then we had **TCP IP**, still available today. We had the **World Wide Web**. And then we had the first **applications** on the World Wide Web.

## slide: "Web developments with an impact on digitization" (Seite 19)

- So we had the World Wide Web. And at the beginning, there was also big misunderstanding whether the World Wide Web really will have a success. For many people it was **just a digital version of a newspaper**. What people at the beginning first understood was that you need to have some kind of <span style="color:red">**reach** and **digitalization**</span>. So some of the programmers, what they did is they put their newspapers then on the World Wide Web to make it accessible there. But what was the **major goal** there? The major **fight was for a URL**. So you could become, you still can do it, but you could become rich by being so early of registering, eg. "spiegel.de" under your name. And then you are before the magazine of Spiegel. And then they want to have the URL and the domain name. And then they pay you a lot of money to get you the domain name. And this is still the case. If you have some cool idea that the new technology is really coming up, reserve the domain name. And then you may become rich by that. Okay, so that was really a **reach and digitalization**.
- And it was <span style="color:red">**reading**</span>, you said it already. I mean, it was what we could do was reading. Read. As the users of the internet **we could just read**.
- And the first things that people just could do was that we set up a homepage. But for that, you needed to be knowledgeable of HTML. And then in the first computer newspapers, you got your first HTML editor. Yeah, that was a cool thing. Because HTML, you could develop your own first **homepage**, homepage toolkits and things like that came up. And that allowed us really to have this kind of thing. And that was the <span style="color:red">**Web1**</span>.
- And the term we coined for the Web1 is <span style="color:red">**"information economy"**</span>. Information was the main thing. And we were getting trying to **get money out of information**.
- And that was also the discussion when people said, well, how can we **earn money** with that? Because **everything is free**. At the beginning, they put all the articles for free because they didn't want to have distribution. Nowadays everything is behind a paywall and needs to be paid. But that was the first thing.
- But we were still <span style="color:red">**consumers**</span>. We just consumed things on the internet.
- And then something happened. There were **big companies** that just **earned money** with **making web pages**.
  - It was a **pixel park**. Pixel park was a famous German company. And they said, our orders, they just came out of the fax machine. Yeah, still. I mean, we have the world wide web, but communication, business communication was still fax. So the orders were just coming out of the fax machine. They earned millions by that. By just producing web pages.

## slide: "Dotcom Crash" (Seite 20)

- And then suddenly there was this big **dot com crash**.
  - And then we need some kind again of radical innovation in this particular thing. So a lot of companies have been completely overrated and completely crashed and things like that. Something similar to being discussed now in the whole AI stuff again.
- But what was happening then, that people had a new idea about the world wide web. And the idea, what you could do is to **search** for something and to read. And then suddenly, we could **interact**. We had something with Wikipedia. Wikipedia and Wikis were the first installations where you could **contribute**. Suddenly, you're not just a reader, you're somehow being a contributor to the web. And the technology was just something like **CGI scripts**.
  - Probably don't know about it. But suddenly there was something like `www.rwth-aachen.de/` and then `input.cgi` and behind that was suddenly a script that allowed you to contribute data. Before it wasn't possible. And that allowed us now to contribute data to the world wide web.

## slide: "Platform Economy" (Seite 21, 22)

- And then we get from there to <span style="color:red">**platforms**</span>. Because suddenly these platforms have been established.
- So we could now <span style="color:red">**write**</span> and now we could write and now the platforms came up, **eg. Amazon, Ebay, Facebook, Google**, and so on.
  - And interestingly enough, most of these platforms had also had their **counterpart in Germany**. In Germany, we had studiVZ. studiVZ was the Facebook of Germany where students interact. We had SchülerVZ, where the school guys interacted. All that was available, but it was completely bought over then by Facebook.
- Wikipedia, Amazon, Ebay, it allowed us to **contribute**. And that was also the time when **YouTube** came out. Or **Flickr**. Flickr was the Instagram of the web1 or web2. Flickr was something where you could upload images.
- And that's the foundation now. That's actually the **foundation for AI** nowadays. Because we contributed a lot of content that is now being used for AI to train the LLMs. We told the LLMs that a cat image looks like a cat because we uploaded pictures and so that it's just detecting that that's a cat. And that's the reason why AI is so powerful at the moment.

## slide: "Web developments with an impact on digitization" (Seite 23)

- But the problem is that we get social media, right? We got digital commerce, right? I feel like .... That's why I said well, we completely rely on that. If they just cut it off, we're gone.

## slide: "Establishing Web3" (Seite 24, 25)

- And then the <span style="color:red">**global financial crisis**</span>. So the three turning points, at least what we found out was, first one **cold war**, data communications. Then, **dot com crash**, we need something new on the internet. And then we go with **financial crisis** where people suddenly never trusted in banks.
  - Suddenly people said well, the whole bank system is completely corrupt. They cheated us. So we have to find something else and just exaggerating that. So it's not that I'm in the case of banks or so. That's actually what happened.
- And then there was the case that people thought about something that can **overrule the banks**. And that was then the <span style="color:red">**Web3**</span>. It was cryptos, it was smart contracts, NFTs, decentralized applications, DAOs and so on.
- And that was then a <span style="color:red">**raising awareness of decentralization**</span>. And as I said, that was 2008, 2007, 2008. And there it happened really on the **banking sector**.
- And what we experience now is that somehow also now happening really on a **data sector**. I mean, people become more and more aware that we are completely **locked into Microsoft, Google, Apple** and all of them. We are completely locked into the system. They're so convenient at the moment they're still cheap to use. We are also **locked into ChatGPT**. We **need a more decentralized approach** to that, which is not that easy. I mean, there have been a lot of approaches.
  - Munich, 20 or 30 years ago, they wanted to get rid of Microsoft and they used Linux.
  - I think now it's Schleswig-Holstein who tries to get rid of Microsoft to put everything on Linux and open source software. It's not that easy in the end.
  - in the Fraunhofer Gesellschaft there was a big discussion whether we go on Teams or not. Then we had Corona and we got on Teams. So it's often these kind of effects. But this is not a big discussion.

## slide: "Web developments with an impact on digitization" (Seite 26)

- So if we follow this up, **information economy**, **platform economy**, and now we call it <span style="color:red">**token economy**</span>.
  - Token economy is because tokens are things that we can register on the blockchain and they allow us to associate a value with a token, put it on a blockchain, make it irreversible, transparent, unchangeable, immutable, whatever you like, and that means we can <span style="color:red">**own**</span> something.
  - And the technology is the <span style="color:red">**blockchain**</span>. Here it was the **world wide web**. Here it was suddenly **active server pages** or **CGI scripts**. Now it's the **blockchain** that brings the new technology behind to **own** something. We're really **writing and owning** something and that makes us from **prosumer** to <span style="color:red">**prownsumer**</span>.
- And there's new people coming up on the sky, which is then suddenly
  - **IPFS**,
  - we have this fox (image) which is **Metamask**, the wallet owner,
  - **OpenSea**, it's a big, big marketplace where you can **own**, where you can trade assets, **NFTs**, pictures and the rest.
- And now we talk about <span style="color:red">**provenance and sustainability**, **authentication**, **automation**</span> and all the rest. So these are the topics we talk about now at the moment.

## slide: "Essential components of Web3"

- So actually, and there are a lot of things now on this particular slide
- the essential component of this Web3 is <span style="color:red">**decentralization**</span>. We aim for decentralization. Make it as decentralized as possible. And I mean, we had already, when we look at <span style="color:red">**peer to peer networks**</span>, a lot of this is based on peer to peer networks. Does someone know when peer to peer networks became really famous, before blockchain? That was the last big hype of peer to peer networks. I could imagine that everybody of you at this time participated in a peer to peer network when you were still at the age at that time.
  - student: I don't know, sharing music.
    - prof: Yeah, sharing music, music sharing, **Napster** and all the music sharing. That was completely peer to peer based. Everybody was running a computer. You had a small server on that one that allowed you to get music stuff from somewhere else.
  - student: **Torrent**?
    - prof: **Torrent is peer to peer**, yeah. That was the origin. And a lot of that is a foundation for blockchain. So peer to peer networks, data sharing directly between users **without central servers**. ... Napster was illegal.
- <span style="color:red">**Smart contracts**</span>.
  - Yeah, we can have code that does some kind of **automation**. We learn that smart contracts have nothing to do with legal stuff and that they're not smart, not clever, whatever, **just some lines of code**, nothing more.
- <span style="color:red">**tokens and cryptocurrencies**</span>,
  - which means suddenly we now have some kind of **representation of assets**, whether it's
    - a **digital asset**, like an image or a kind of a coin, or
    - some kind of **real assets** like saying, well, this particular chair is currently being owned by the university by just mapping that onto the token.
- We can <span style="color:red">**build ecosystems**</span> now.
  - We can have real **chains of things**. So I can now have a **supply chain**. I can say, look, I have a token about the origin of this particular piece of wood. And this is coming from a forest that is not being cut in a rainforest and so on. And that carries along the whole ecosystem.
- <span style="color:red">**Decentralized apps**</span>
  - are really **based on smart contracts**. They have **no server**. They **run completely on the blockchain**. So it's not this kind of Django (back-end server side web framework) stuff, server-client or so. It's completely based on the blockchain.
- And we discuss about <span style="color:red">**governance**</span>.
  - How do we govern such a blockchain?
  - The question that you often ask is "who determines what's happening on the blockchain?". Is it the user? Is it the community? And that's also something that we are going to discuss.
- So these are things that are relevant for the Web3.

## slide: "Lecture Objective"

- And this is what we actually then do within the lecture.
- So after today's very broad introduction into what it's all about, the next lecture will cover some kind of foundation.
- So we talk about
  - hashing,
  - public and private keys,
  - what is a **wallet** and
  - how is **Bitcoin** actually working?
    - We start with Bitcoin because it's just requested to deal with, and
  - you learn what the **proof of work** is,
    - why Bitcoin is **energy intensive**,
    - where bitcoins come from,
    - where are they being **mined**,
    - so the big mysteries.
  - And then we also discuss some kind of **use cases** that can be on the blockchain.
    - So foundations is more IT stuff, and then use cases before we discuss some more application stuff,
  - some kind of **blockchain implementations**,
    - because there is not "the" blockchain, there are a lot of different blockchains.
  - And very interesting at the moment are **second level chains**,
    - so chains that are built upon other chains. So we have a hierarchy of blockchains in the meantime, which makes it extremely scalable, cheap to use, often saying, well, blockchain doesn't cost anymore anything.
  - We talk about **Web3 applications**
  - and also the **economy of things**.
    - And when we talk about the economy of things, we talk about **weather stations**, we talk about **LoRaWAN (a network protocol, long range wide-area networking) stations** and just the approach to earn money by just operating a weather station. And this is going back, you said, really spreading the benefit of something equally. And this comes back to that. That you own something, you own a device, and this device is producing data and you are being paid by sharing this data with other people. And that delivers tokens to you, which are extra kind of data.

## POAPs

- Okay, so now you can get your first **token**. This is something which is a so-called **POAP**, POAP stands for **"proof of attendance protocol"**. Proof of attendance protocol means that you can **prove that you have been here**.

...

- I have met Wolfgang and then we meet each other and when you also collect POAPs you just scan this and then you have **proof that you have met me**.
- So it sounds strange.
- **What is it good for?**
  - We are already getting into the discussion on applications.
  - On the one hand it's just nice. We just **collect** a lot of things.
  - So on the other hand what I have now is a **collection**.
    - So maybe you are not gonna see it, but that's a collection of all the POAPs that I got. So it's like a **sticker book**. It's my digital sticker book. No value. But, we can **associate value with that**.
    - So I got, for example, from some companies I got the sticker, I got stickers from some people from Bayer. Bayer is using it heavily. So Bayer is a pharmaceutical company. They use it for all the seminars. So when you participate in a particular seminar you get it and they just use it as a proof that you have been there. It's kind of a certificate.
    - But what I could also do is, this was just a proof of the attendency that you have been there. Next time what I can do is that I ask a very difficult question and the clever boy or girl of you who is answering it, I just show him the QR code, he scans it or she scans it and then you get the **POAP for answering the most difficult question** in the web3 lecture.
- Next thing is that I say, OK, I would like to **get you some goodies for those of you who have participated in the lecture 90% of all the time**. For those, they **get access to a particular web page where you find some very important information for the exam**. So how do you get there? You have to look in with your wallet. Then by accessing this, you provide your address, your wallet address, your identity. I can check with the program that you have at least eight of ten POAPs and then I give you access to the web page. So, this is then called **"token-gated access"**. You get access to something because you have a token. Sometimes people do that at a fare. If you have a VIP pass, you get a coffee if not you're outside.
- With **NFTs**, it's already a story for another lecture, but with NFTs, people have entrance to a particular concert.
  - By having a certain kind of NFT, you could participate in a concert of a famous rock star. It's like an entrance ticket. It's like a certificate. That's only available for those.
- Also, something is that this POAP, as I said, **you can copy it** and publish it on Twitter or whatever and then even somebody in Russia, for example, could just take this POAP and claim that he participated in this lecture.
  - solution: One thing is that they can be **rolling**. It's a website. It's not just a copy of that one. And you have to hurry up because after four o'clock, they are no longer valid. So if you copy it after four o'clock, it says the event is over. Sorry.
  - So all these things are possible. But you have a question?
    - student: Yeah, I know you answered already. I was like, okay, that's cool and all, but what if I'm staying at home and someone from here just sends me a picture?
      - prof: That's possible. And I think in one of the next lectures, we just do it as an exercise, to create POAPs for some for the other ones. Or we just do it here together just to create one so that you see how it works. It's not that difficult. I think it's a nice stuff.
- There has been some kind of **hype** around it. At the moment, it's going a little bit down. There was a big, big discussion about POAPs. Even Vitalik Buterin, who is founder of the Ethereum blockchain, he promoted it, having this whole idea of indicating that you have been somewhere by getting such a POAP.
- So I really recommend downloading the **app** because then it's much easier and then in the app you can collect all the NFTs of the lecture. So you have a lecture collection.
  - student: I saw that the app basically just manages your wallet for you. You have your wallet. But technically you can use **Metamask** for example, if you have that address in the wallet.
    - prof: Yeah, you can do so. **Metamask** is really clumsy in the end. But if you use Coinbase, Coinbase can understand, if you use for the POAP the same address that you also have registered in the Coinbase wallet, the same public address, then Coinbase will list all the POAPs. Then suddenly they pop up also in the Coinbase wallet. So there's a lot of interoperability in the new app. So all that is possible there. Yeah, good.

## org

- Okay, so what's all about the lecture Wednesday, tutorials bi-weekly.
So next lecture you will get some kind of an exam, I don't know, exam, like access time.
And then we'll discuss it the next week.
There is Moodle, there's a Moodle space.
There are upload slides, these slides, and there are also other kinds of material.
And other kinds of readings and material.
And during the course I will also announce the time for the final written exam.
And if you pass the exam, you may also get it.

## student questions

On the Arbettian Arc and the lectures until the...
Yeah, it's just... I think we wouldn't need all the time.
Sometimes I may stop the lecture in the area.
It's how it's fluent. So we just stop the lecture and then if you like you can get something to train out of the rest of the lecture.
What would be possible to upload the slides before lecture?
Yeah, can use all that.
Thank you.
The good thing is, or the bad thing is, I'm not here next week.
So next week is cancer.
There are a lot of professors who are not here next week.
I mean normally the lectures often start only next week.
But I thought, well, let's start because if I skip it two times...
There are an exam admission.
Sorry again.
There is an exam admission.
In 10-4 hours.
Yeah, in 10-4 hours.
Oh, Bitcoin?
No.
That's an exam admission for example.
Yeah, any questions?
Are there homeworks that we have to do?
Yeah, homeworks. That's what I call for the exercises.
So that was the exam homework.
And this will be maybe sometimes to read something.
That's really a bit of a lie and such that we can discuss it.
Maybe watch a video on programmes for our country.
Things like that.
Or just some other kind of programming.
The exercises, for example in some courses, they come towards some bonus points.
Yeah, I have to think about that.
I know that I gave this lecture in the past and she has to go to Stomach's house and he retired at the meantime.
And she did that and that very often led to some strange situations in the end.
I think there will be at least a few submissions.
So, two submissions of the homework.
But not to attend the exam.
Yes, to attend the exam.
Okay, then that's the prerequisite for the exam.
Yeah, it doesn't count for the exam.
Yeah, so the exam is the exam.
That's a cat in my mind.

prof: Yeah, a programming. Are you all capable of programming.
student: What language?
How will?
Well, I mean, it's sometimes interesting just to do some kind of...
I mean, it will be somehow a must when we talk about smart contracts because they are a good program.
And you're a computer scientist, so I hope that you can at least deliver the program.
Maybe for one or two homeworks it's just that you access an API, you get some data and you realize it's a good one.
Well, I mean, actually sometimes...
I mean, we have some difficulty at the most.
I give you a task to access an API and to write some kind of output in Python from writing a good prompt solves the problem.
I know that. That's how it's what I'm doing.
So, why should we use it?
Nevertheless, it's good to do it because at least you see what's going on.
Or when you talk about the **interplanetary file system**, the **IPFS**, which is very counterpart to the blockchain and used for a lot of use cases.
It's also something...
I mean, understanding what it's all about.
Sometimes it's easier to discuss with the chatGPT you can do instead of reading a paper.
Nevertheless, I think you should at least know some of the basics how research actually goes.
And if you would like to produce code and an exercise is chatGPT you can do fine.
I don't care about this.
Because actually it's the process that is what you learn and the result.

## POAPs

- Ah, still, with POAPs, they also form some kind of **social network**.
  - Now, once you got your first POAP and once you have really registered your existence, so you got a wallet and an identity, you can see how many people own this particular POAP here. And then you see those people who own this POAP. Which other POAPs do they own? And then you will see that there's one user who owns a lot of other POAPs, which is me. Which is then actually a nice indication of a **social network**. Suddenly you get a social network of people who have been present at a certain location or at a certain event.
  - And we did a master thesis on that. That someone just pulled all the data. He really retrieves all the data from the POAP system. He completely retrieved it and then he made some nice **social graphs** on one of the most famous events. And you will see some kind of basics there. We will discuss it later on.
- But you can check now to see what's happening with this particular POAP here.
- student: Could you show it again?
  - prof: Yeah, I can show it again.
- So what it says is, it says one out of eight. So it says eight people have collected. You and 27 more were there. So at least 26 people did this. And the cool thing is most of you can already see there.
- So there are already a lot of people who collected it with an address. So not just collected it with an e-mail address, but collected it already with an address. So we have eight **mints**. Minutes means you really **minted** it. You have a public address and you minted it to your address. And you have 20 e-mail reservations, which means 20 people just reserved it by e-mail. And as soon as you download the **wallet**, the app, and register it, then your e-mail reservation becomes a **mint**.
- And that is also what the student analyzed. What is the ratio normally? So if I do this, for example, at a business conference, you have business people sitting there and really to get this into the head that there is something they should look that, for example, for audio systems or things like that, they normally scan it and then you see e-mail reservation. And the turnover from an e-mail reservation to a real mint is a factor really on how Web3 are you. So if after the whole lecture, there are still 20 e-mail reservations, so in three months, I did something wrong. So I didn't convince you to go into this direction.

## student questions

student: Do we have to download the app?
Is it necessary to download the app?
It's necessary to download the app.
Because with the app, you actually get an ID, an identity.
You get a key.
So we do need the theorem wallet in the app, right?
You need the poor wallet.
Just the poor wallet.
But you could also associate it later on with the theorem.
Because the e-mail set the poor wallet.
Yeah, maybe then they changed something.
I did it about several years ago.
How does this registration?
Which registration?
For the **minting**.
prof: The minting. We discuss next time.
Ah, okay.
Because this means that really the token is generated and things like that.
So if you have this minted on it, we have these options.
Different options.
Ah, okay.
prof: **Minting** means that you really mint it to your wallet.
Ah, we need to connect to the token.
Yeah.
Sven?
Yeah.
Can you also see how many people have minted the very clear in all those coins?
Yeah, that's at least what I think here is that we have eight mints.
At least these people have been walleted.
Can you see how many theorem points are connected to that?
Oh, but you have to read them.
So, yeah.
Something else.
I wanted to highlight.
No, it's gone.

## org

For the next course which will be on today, will we have a exercise for then?
Sorry, no, it's coming to the game then.
We'll have exercises to do for the next one.
No, for the next one.
And next week there is no lecture.
I also have a question about the lecture.
Like it was the other page, like it was online.
It changed from 5 to 4.
Can you verify it?
Like is it like 4 pages for the course?
Oh, I have to check.
Yeah.
Since it's a two lecture, two lecture, one hour exercise lecture.
So it's a 2.1.
Some of us have to check how much it works.
So currently it's like 14 minutes.
This is far could be.
I can't see the tutorial.
No, on the calendar I couldn't.
I will announce them.
Okay, and there is a location for the tutorial.
Sorry, again, location.
Oh, here.
I'll give you all the details.

## Blockchain Club

Yeah, and if you...
Ah, that's all right.
Just in case that you think you would also spend some kind of free time.
Who is this?
You are there?
I'm actually at the club for a year now.
Yeah, you are Mr. Fahram.
I look, you are.
There are some students of mine who founded it.
And some others who have been associated with us.
Actually, everyone has told me about the course.
Yeah, he is working with us.
Because I think he is the chair of...
Mike.
Mike was a student of mine in the prosimina.
Then he did the bachelor thesis.
And he was a good teacher.
Yeah, yeah, he finished.
He just did this master thesis.
So what they do is they organize a lot of different events.
So they organize a lot of different events.
I advise.
There is a lot of free food all the time.
They have free food from some of the blockchain companies.
So there is a lot of funds.
It's also really good for networking.
So a lot of people who actually are from the club,
just do positions in the club.
They get internships in companies or even companies that are interested in them.
Just from conferences, networking and stuff like this.
So it could be a really nice entry like gateway into blockchain in Germany.
So for example, I'm going next week to a Solana hackathon just through the club.
So even though I don't know much,
it's just a little bit through signing up to the club.
Yeah, so the hackathon is...
Okay, that's unfortunately just in German.
That's the homepage of our particular project here.
And there you will...
It's in German because it's addressing German companies basically.
But there are also events and some other things that you can look up there.
The show on demonstration.
I will upload more information about places to go or places to go or to see during the next one.

## org

The first tutorial is going to be in two weeks, right?
Not the tutorial, no?
In two weeks we have a lecture and then I will give you some homework and then we have to talk.
The first one will be earliest in three weeks.
Okay.

## Blockchain Club

Well, I think we're doing onboarding in two or three weeks and we only do it once a year.
So if people really want to join, I think it's the chance to work like that.
I'm not sure. You can't write something in Google, can you?
Me? I don't know.
Otherwise you can just send a message to me about something that is interesting and then I can just put it up on Google.
Sure.
There are a lot of hackathons in this area.
It's really the case that all these business, all these blockchain companies, they really have a lot of money.
They really have a lot of money because they earn a lot of money and they really put it back into the community
because they want to raise it.
Zolana is not doing that for free. They want to have good students and they want to have advertisers.
That's why you get to put something.
The price pool is 500,000.
Actually, friends of mine that I took my first blockchain course, they attended the hackathon and they won 20,000 US dollars.
Also here, they also participate in the hackathon, so they also put back this first price.
So there's a lot of money.
But normally you won't get it in euros, you get it in tokens.
So you have to have a wallet to get it back.
The last month we did actually two events with like 1,500 dollars price pool and another one of those.
So there's always money. I'm surprised. It's a student club, but like...

prof: That's also something. If you come up with some kind of hackathon, I'm really free that we do this as some kind of tutorial.
So if you come up, like you say, well, there was a hackathon in January or December or late November,
otherwise it's late, and that's the following topic.
I'm really happy that we spend, for example, one lecture on just discussing that.
Not discussing this so that I can provide some input and they have some wisdom of the crowds to say,
well, what kind of application could be better there?
So I was really open for that one.
Okay, so then thanks for participating in the first lecture.
I hope to see you then in the next one.
Yeah, yeah.
Yeah.
Yeah.
