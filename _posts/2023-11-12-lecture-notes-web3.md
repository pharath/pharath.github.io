---
title: "Web3 and Distributed Ledger Technology"
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

# Introduction

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

- So we had the World Wide Web. And at the beginning, there was also big misunderstanding whether the World Wide Web really will have a success. For many people it was **just a digital version of a newspaper**. What people at the beginning first understood was that you need to have some kind of **reach** and **digitalization**. So some of the programmers, what they did is they put their newspapers then on the World Wide Web to make it accessible there. But what was the **major goal** there? The major **fight was for a URL**. So you could become, you still can do it, but you could become rich by being so early of registering, eg. "spiegel.de" under your name. And then you are before the magazine of Spiegel. And then they want to have the URL and the domain name. And then they pay you a lot of money to get you the domain name. And this is still the case. If you have some cool idea that the new technology is really coming up, reserve the domain name. And then you may become rich by that. Okay, so that was really a **reach and digitalization**. And it was **reading**, you said it already. I mean, it was what we could do was reading. Read. As the users of the internet **we could just read**.
- And the first things that people just could do was that we set up a homepage. But for that, you needed to be knowledgeable of HTML. And then in the first computer newspapers, you got your first HTML editor. Yeah, that was a cool thing. Because HTML, you could develop your own first **homepage**, homepage toolkits and things like that came up. And that allowed us really to have this kind of thing. And that was the **Web1**. And the term we coined for the Web1 is **"information economy"**. Information was the main thing. And we were getting trying to **get money out of information**.
- And that was also the discussion when people said, well, how can we **earn money** with that? Because **everything is free**. At the beginning, they put all the articles for free because they didn't want to have distribution. Nowadays everything is behind a paywall and needs to be paid. But that was the first thing.
- But we were still **consumers**. We just consumed things on the internet.
- And then something happened. There were **big companies** that just **earned money** with **making web pages**.
  - It was a **pixel park**. Pixel park was a famous German company. And they said, our orders, they just came out of the fax machine. Yeah, still. I mean, we have the world wide web, but communication, business communication was still fax. So the orders were just coming out of the fax machine. They earned millions by that. By just producing web pages.

## slide: "Dotcom Crash" (Seite 20)

- And then suddenly there was this big **dot com crash**.
  - And then we need some kind again of radical innovation in this particular thing. So a lot of companies have been completely overrated and completely crashed and things like that. Something similar to being discussed now in the whole AI stuff again.
- But what was happening then, that people had a new idea about the world wide web. And the idea, what you could do is to **search** for something and to read. And then suddenly, we could **interact**. We had something with Wikipedia. Wikipedia and Wikis were the first installations where you could **contribute**. Suddenly, you're not just a reader, you're somehow being a contributor to the web. And the technology was just something like **CGI scripts**.
  - Probably don't know about it. But suddenly there was something like `www.rwth-aachen.de/` and then `input.cgi` and behind that was suddenly a script that allowed you to contribute data. Before it wasn't possible. And that allowed us now to contribute data to the world wide web.

## slide: "Platform Economy" (Seite 21, 22)

- And then we get from there to **platforms**. Because suddenly these platforms have been established. So we could now **write** and now we could write and now the platforms came up, eg. Amazon, Ebay, Facebook, Google, and so on. And interestingly enough, most of these platforms had also had their counterpart in Germany. In Germany, we had studiVZ. studiVZ was the Facebook of Germany where students interact. We had SchülerVZ, where the school guys interacted. All that was
available, but it was completely bought over then by Facebook. Wikipedia, Amazon, Ebay, it allowed us to contribute. And that was also the time when YouTube came out. Or Flickr. Flickr was the Instagram of the web1 or web2. Flickr was something where you could upload images.
- And that's the foundation now. That's actually the foundation for AI nowadays. Because we contributed a lot of content that is now being used for AI to train the LLMs. We told the LLMs that a cat image looks like a cat because we uploaded pictures and so that it's just detecting that that's a cat. And that's the reason why AI is so powerful at the moment.

## slide: "Web developments with an impact on digitization" (Seite 23)

- But the problem is that we get social media, right? We got digital commerce, right? I feel like .... That's why I said well, we completely rely on that. If they just cut it off, we're gone.

## slide: "Establishing Web3" (Seite 24, 25)

- And then the **global financial crisis**. So the three turning points, at least what we found out was, first one **cold war**, data communications. Then, **dot com crash**, we need something new on the internet. And then we go with **financial crisis** where people suddenly never trusted in banks.
  - Suddenly people said well, the whole bank system is completely corrupt. They cheated us. So we have to find something else and just exaggerating that. So it's not that I'm in the case of banks or so. That's actually what happened.
- And then there was the case that people thought about something that can overrule the banks. And that was then the **Web3**. It was cryptos, it was smart contracts, NFTs, decentralized applications, DAOs and so on. And that was then a raising of **awareness of decentralization**. And as I said, that was 2008, 2007, 2008. And there it happened really on the **banking sector**.
- And what we experience now is that somehow also now happening really on a **data sector**. I mean, people become more and more aware that we are completely locked into Microsoft, Google, Apple and all of them. We are completely locked into the system. They're so convenient at the moment they're still cheap to use. We are also locked into ChatGPT. We need a more decentralized approach to that, which is not that easy. I mean, there have been a lot of approaches.
  - Munich, 20 or 30 years ago, they wanted to get rid of Microsoft and they used Linux.
  - I think now it's Schleswig-Holstein who tries to get rid of Microsoft to put everything on Linux and open source software. It's not that easy in the end.
  - Fraunhofer Gesellschaft there was a big discussion whether we go on Teams or not. Then we had Corona and we got on Teams. So it's often these kind of effects. But this is not a big discussion.

## slide: "Web developments with an impact on digitization" (Seite 26)

- So if we follow this up, **information economy**, **platform economy**, and now we call it **token economy**.
  - Token economy is because tokens are things that we can register on the blockchain and they allow us to associate a value with a token, put it on a blockchain, make it irreversible, transparent, unchangeable, immutable, whatever you like, and that means we can **own** something.
  - And the technology is the **blockchain**. Here it was the **world wide web**. Here it was suddenly **active server pages** or **CGI scripts**. Now it's the blockchain that brings the new technology behind to **own** something. We're really **writing and owning** something and that makes us from **prosumer** to **prownsumer**.
- And there's new people coming up on the sky, which is then suddenly **IPFS**. We have this fox which is **Metaverse**, the wallet owner, **OpenSea**, it's a big, big marketplace where you can own, where you can trade assets, NFTs, pictures and the rest.
- And now we talk about **provenance and sustainability**, **authentication**, **automation** and all the rest. So these are the topics we talk about now at the moment.

## slide: "Essential components of Web3"

- So actually, and there are a lot of things now on this particular slide, the essential components
of this map three is decentralization. We aim for decentralization, make it as decentralized as
possible. And I mean, we had already, when we look at peer to peer networks, a lot of this is
based on peer to peer networks. That's how we're known when peer to peer networks became really
famous before blockchain. That was the last big hype of peer to peer networks. So I could imagine
that everybody of you at this time participated in the peer network when you were still at the age
at that time. I don't know, sharing music was true too. Yeah, sharing music, music sharing,
Napstar and all the music sharing. That was completely peer to peer based. Everybody was running a
computer. You had a small server on that one that allowed you to get music stuff from somewhere else.
That was the origin. And a lot of that is a foundation for blockchain.
So peer to peer networks, data sharing directly between users without sample servers.
Now we have.
Sorry.
Smart contracts. Yeah, we can have code that is the, that does some kind of automation. We
learn that smart contracts have nothing to do with legal stuff and that they're not smart,
not clever, whatever, just some lines of code, nothing more.
It's tokens and group accountings, which means suddenly we now have some kind of
representation of assets, whether it's a digital asset, like an image or kind of a coin,
or some kind of real assets like saying, well, this particular chair is currently being owned
by the university by just mapping that onto its own. We can build ecosystems now. We can have real
chains of things. So then I can now have a supply chain. I can say, look, I have a token about the
original of this particular piece of wood. And this is coming from a forest that is not being
cut in a rain forest and so on. And that carries along the whole ecosystem. Decentralized apps
are really based on smart contracts. They have no server. They run completely on the blockchain.
So it's not this kind of jungle stuff server client or so. It's completely based on the
blockchain. And we discuss about governance. How do we govern such a blockchain? The question
that we often ask is who determines what's happening on the blockchain? Is it the user?
Is it the community? And that's also something that we are going to discuss. So these are things
that are relevant for the Web 3. And this is what we actually then do within the lecture.
So after today's very broad introduction into what it's all about,
the next lecture will come out some kind of foundation. So we talk about hashing public
and private keys, what is more that and how is Bitcoin actually going to be? We start with
Bitcoin because it's just interesting to do with, as you learn what the proof of work is,
why Bitcoin is energy intensive, where bitcoins come from, where are they in mind,
so the mysteries. And then we also discuss some kind of use cases that can be used in blockchain.
So foundation is more IT stuff, use case before you discuss some more application stuff,
some kind of blockchain implementations, because there is not the blockchain, there are a lot of
different blockchains. And very interesting at the moment are second level chains, so chains that
are built upon other chains. So we have a hierarchy of blockchains in the meantime,
which makes it extremely scalable, cheap to use. It's often saying, well, blockchain doesn't cost
anymore, anything. We talk about that three applications and also the economy of things.
And when we talk about the economy of things, we talk about weather stations, we talk about
low-route stations and just the approach to earn money by just operating a weather station.
And this is going back, you said, really spreading the benefit of something equally. And this comes
back to that. That you own something, you own a device, and this device is producing data and you
are being paid by sharing this data with other people. And that delivers tokens to you, which are
extra kind of data. Okay, so now you can get your first talk. This is something
which is a so-called pro-app, pro-app stands for proof of attendance protocol.
Proof of attendance protocol means that you can prove that you have been here. And we are to do
it.
