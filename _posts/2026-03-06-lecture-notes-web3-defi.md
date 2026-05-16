---
title: "Web3 and Distributed Ledger Technology - DeFi"
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

And as usual, just try to raise your hands or just try to interrupt me.
So then, organize the key on the screen so I'll see those of you.
But sometimes maybe I forgot that I'm here in the lecture and alone I need to do recording.
Okay, good. So now the recording started.
Welcome to this lecture on decentralized finance.
Some of the concepts have already been discussed during the lecture in some examples.
But today we have another more comprehensive look at the whole thing together with some examples.
And afterwards then we can discuss a few questions that you have about the lecture and preparation for the exam.
Good, decentralized finance.
You can see it in any other kind of newspapers.
It's defi, it's mentioned to be the new approach towards financing.
And what we can do here with financing is basically, if we look at the different basic concepts,
it's based on smart contracts.
Another term that we hear about in this context is decentralized exchanges.
Sometimes it's a bit of a ruckus because some people are joining and I just have to acknowledge them.
And then we discussed a little bit about lending and borrowing.
Stablecoins, we discussed that already, yield farming and the rest are more some terms that we don't go into much detail.
Okay, so let's have a look first at some basic concepts.
The first thing actually here is that of a smart contract.
I mean, decentralized finance is only possible because of smart contracts.
I mean, just based on the Bitcoin blockchain, there wouldn't be any defi concept.
With Bitcoins, you basically just would have a central exchange where you can exchange Bitcoins to any kind of fiat money.
But this is then done by a central exchange.
And all these central exchanges, they are based on an order book.
There are some people that offer Bitcoins and other people offer euros in exchange for that.
And when I say I would like to sell my Bitcoins for a thousand euros per Bitcoin, this can happen.
It's just part of the order book and then a lot of people will say, yeah, bargain, let's buy it.
And if I want to sell them for 200,000 euros, it's in the order book, but probably nobody will buy it unless there is a big, big need to buy anything of that.
That is basically what these normal order books do.
With the whole issue of smart contracts, people thought about other ways to work or to provide a way to exchange different crypto tokens or to exchange different kinds of assets.
So as you see here in the example, it's that actually here we have, where's my ledger pointer here?
Now it's back here.
So we have the example, a loan management that automatically collects collateral if the debitor falls below the minimum value.
Or another smart contract, this is that automatically the smart contract is connected to a digital lock that is fixed at the door of your student room.
And if you don't pay, I see this comment, let's see it later on.
So if you fail to pay your rent, then the lock doesn't open anymore.
Okay, so let me see. There was a chat comment.
There are also decentralized swappable beads.
Yeah, okay, that's right.
But again, they use some kind of additional chain to do that through the Tor chain, as you say.
But what we need something that is an addition then to the normal Bitcoin blockchain.
And that came in particular through the smart contracts.
Okay, so decentralized changes. What's the point here in a decentralized change?
We have a training between the users.
There is no central custody.
And that's really the trick behind it.
When you want to exchange on the normal way to get your crypto is that you go to Bitpanda or CoinDesk or any of these big central platforms in order to get your crypto.
And here we don't have this kind of central custody. There is not this platform.
All we have is basically a smart contract.
And if we have this smart contract, there is no person who's controlling autobooks.
This can also be done automatically, of course, using some kind of algorithms.
But we need some very simple way to do that in a smart contract.
And that is then done with this so-called automatic or automated market makers, AMM.
And these are particular kind of models that are separate from the traditional autobooks that we know from stock exchanges or the stock exchange.
And there are some important terms that you should remember.
On the one hand, it's a liquidity pool.
If we want to provide a smart contract where we can exchange, for example, ETH towards USDC, so ETH towards a stablecoin or whatever,
then we need liquidity.
So we need a certain amount of both tokens.
So we need a certain amount of both tokens such that the exchange can take place.
So if we don't have these kind of liquidity pools, we can't exchange.
So we need this kind of pool where we see the money and then we can exchange the two different tokens against each other.
Then there's something else, which is another term that you will come across when you go into these different systems.
It's the term of impermanent loss.
And the impermanent loss is some kind of risk that you as a participant of a liquidity pool has if the pool changes significantly.
And it's not a real loss.
So you somehow don't really lose money in the sense that you lose money compared to if not investing in a liquidity pool.
And we have an example for that on one of the next slides.
So let's imagine now what happens in these kind of liquidity pools.
Let's imagine that we would like now to start a liquidity pool.
What we need to do is to have a smart contract.
And in this smart contract, we put up drop to currencies.
So each of us is now putting drop to currencies into this particular smart contract.
So you take up your wallet and then you submit your money, let's say 10 ETH or 1000 USDC to this particular liquidity pool.
And the buyers and the sellers on that are not matched.
But at the beginning, we just put money into this liquidity pool.
And this liquidity pools, they always have two or more tokens.
So in the main, almost often they have two.
So ETH or USDC or ETH and Bitcoin or whatever.
And the big magic behind that, and even touched this already last time and touched it already briefly in one of the previous lectures,
is that the automatic market maker uses a particular formula.
X times Y results in K.
So X is the amount of the first token, let's say ETH.
Y is the amount of the second token, USDC.
And K is a constant that doesn't change.
And this constant is computed when we install the liquidity pool for the first time.
And the question for you is, what's in it for me? Why should I do this?
Well, you get trading fees.
So there's no free lunch.
If you want like to exchange crypto tokens, for example USDC against ETH,
then you have to pay a trading fee, which is on average 0.3% per swap.
And the revenue increases now with the trading value.
The more people trade in your particular pool, the more you can earn as an offeror of liquidity by the trading fees.
And this is what is often called passive income.
So you provide money to a liquidity pool and then you receive passive income by the trading fees.
You could also compare it to interest rates.
So interest rates are also some kind of passive income.
You don't really have to actually work for it.
You just provide your money for that.
Now, let's look at an example to see how this actually works.
Let's imagine now that we start our liquidity pool.
And we all put together 10 ETH and 20,000 USDC in the pool.
This is based on the assumption that at the moment one ETH is worth 2,000 USDC.
In particular, 2,000 USDC.
And we deal here because we can't trade fiat money there.
We trade the representation of the fiat money, which is the stable coin, the USDC.
So for one ETH we put 2,000 USDC in our pool and in the end we have 10 ETH and 20,000 USDC.
So now we apply our formula with the automatic market maker.
X times Y is 200,000.
So 10 times 20,000 is 200,000.
And this K now remains stable.
When we trade something, there's always the multiplication.
When we take ETH out in exchange for USDC or vice versa,
the result of both in our pool must remain 200,000.
And since we exchange somehow the amount, what we somehow adopt and modify,
is the value of each asset in the liquidity pool.
And you see this on the right hand side.
So let's first, you see my value is now, let's see, I'm the only one or you are the only one who's running the liquidity pool.
Your value is then currently 40,000 USDC.
So it's 20,000 USDC plus 10 times 2,000.
It's altogether 40,000 USDC.
That is your value that you have.
And this is what we need somehow to check the permanent loss.
Now, we have a first trade.
A trader buys ETH with 1,000 USDC.
So now someone is coming and say, I would like to have ETH and I'm providing 1,000 USDC.
So before, again, we had 10 ETH, 20,000 K, 200,000.
After the deposit, the pool contains 21,000 USDC.
Because someone transferred 1,000 USDC to our pool, the sum is 21,000 USDC.
So now we have to adopt and now we have to pay out ETH for these 1,000 USDC.
And the initial guess is that, okay, I pay 1,000 USDC.
And you would expect that you get 2,000, that you get 0.5 ETH.
Because here, based on our initial formula, 1 ETH is worth 2,000 USDC.
So 1,000 USDC is worth 0.5 ETH.
So that is what you would expect as an outcome of the trade.
But this doesn't happen.
You don't get exactly that amount.
Because what we need to do now is we need to calculate the new value of ETH.
And the new value of ETH is determined by taking our constant 200,000,
divided by the new amount of USDC, 21,000, which results in 9.5238 ETH.
So the ETH value is now being adopted, such that when we multiply it with the ETH amount,
with the USDC amount, that in the end we sum up with 200,000.
So the trader now receives the difference between 10 and this particular 9.538,
which is 0.47.
So this is what the trader receives.
The trader does not receive 0.5, what you would normally guess.
You only get 0.4762.
Because that is the new value of ETH.
And in order to keep our whole liquidity pool equal,
so in balance with the constant of 200,000, that is what we get.
So the new ETH price is then 2,200 and 5, it's becoming more expensive,
because we have less ETH in the pool.
So before it, we had 10 ETH in the pool and now it's just 9.5.
So the value now of my previous value was now that I was owning 40,000.
If I would have taken, if I would have not invested these money into the liquidity pool,
so imagine you were running this liquidity pool and I have this amount in my normal wallet.
And then now I observe that this liquidity pool is now raising, is being in exchange,
and suddenly the ETH price is being increased.
Then I can calculate what's the count value and then my count value would be 42,000.
Because suddenly ETH is becoming more expensive and therefore my value would be 42,000.
So if I would have been holding my money in my pocket, I would now own 42,000 instead of just 40,000.
So this is then somehow the impermanent loss or the impermanent increase.
So somehow it increased, my money got more value because the market has changed.
So that is more just a side effect.
The most important thing is somehow that you guess or that you get the idea what is happening here in this liquidity pool
based on this formula X times Y must remain constant.
And so we adopt the value here and this results then in the new exchange rate.
But I see a question there. Please go ahead.
So if I understood this correctly, then the person that would hold onto the money
and does not open a liquidity pool would in the end have more money than my liquidity pool
because I still got the 40,000 USDC worth of assets.
But the person that starts with the same amount of tokens and then keeps the money suddenly has more value.
So why would I want to open a liquidity pool in the first place?
I know there are the exchange rates, but do they need to be calculated in a way that my loss,
because impermanent loss is then weighted up against or?
Yeah. So on the one hand you would do that in order to earn the trade fees.
So that is one thing.
On the other hand, in this particular case here, this is a very small liquidity pool.
So small exchanges like 1,000 USDC is already, let's say, 5% of the overall pool.
So I have chosen this example really to have some kind of immediate changes here in the exchange rate.
Normally the liquidity pools are much, much larger.
Therefore the value does not change that rapidly.
Therefore the danger of this impermanent loss is not that big.
So on the other hand, the danger of impermanent loss is high.
If you really invest in cryptocurrencies that have a high volatility,
or into liquidity pools that are not very big, so that you have an immediate imbalance of the overall pool.
You see here the other example that you see in gray, that that's our old transaction.
And now we trade one ETH for USDC.
So now we have the opposite.
Now someone is coming along and he's giving one ETH and he wants to get USDC.
So before we had 9.538 ETH with this, OK, remains 200,000.
And after the deposit we have 10.5 ETH.
So 9.5 plus the one that the trader provides.
So the new USDC quantity is now 200,000 divided by this new amount of ETH, which is 90,000 for USDC.
And therefore the trader now receives the difference between the 21,000 USDC and the 19,000 for USDC, which is 1,999 something.
So again, it doesn't receive 2,000.
But we thought at the beginning, it receives less.
We have a new ETH price and now ETH becomes cheaper.
And suddenly the model is 39,000.
So here in that case, suddenly you're losing money by not investing in the liquidity pool.
So you see, this goes up and down always.
So it's really somehow an investment game that you do there.
And on the other hand, what Ivan told you already last time, if for example one of these liquidity pools has a high fluctuation, so it becomes very volatile.
And if the value of ETH suddenly becomes very cheap or very expensive, what happens is that people exchange money at this liquidity pool, which has another exchange rate and invest it there.
So there is a high fluctuation then between the liquidity pools in order to keep them in balance, because people want to try money by arbitrage between the different liquidity pools.
And this now leads to the fact that we have a lot of trade, which is okay for you as an investor into a liquidity pool.
So for example, that the liquidity pool providers now own shares in the pool, let's say 10%.
And with ETH exchange, they earn a proportional fees.
So for example, 0.3% per trade.
So if we have two trades of 1000 USDC and one ETH, they would generate.
So the two trades that we have just done in our example, they generate approximately 9 USDC as fees.
And these are now automatically distributed to the liquidity pool providers.
And if you own 10% on the liquidity pool, you would get 0.9 USDC.
So as a provider of liquidity, you benefit from the trading volume, not from the price direction, where that goes up or down, you don't care, you just benefit from that.
So I'm not trying to convince you here to invest in any liquidity pools.
I'm not investing in any liquidity pools, but this is just to explain to you how that works.
How this automatic market makes us work.
And on the other hand, the whole, I wouldn't call it philosophical, but the whole idea behind this is that they say, okay, we can have a completely automated market,
which is completely based on algorithms, which is not based on any bankers who manipulate the exchange rate or do any other kind of thing.
So we are not relying here on any kind of platforms or whatever.
It's completely regulated by a smart contract that handles everything.
So it's complete automation of the overall exchange of different kind of crypto-tones.
So this on the liquidity pools and automatic market makers, and I think what you should, I see your question,
what you should remember also for the exam is the overall mechanism.
So the overall mechanism of this automatic market maker, so the formula, and in particular also,
if you look at this for the first time, the somehow surprising effect that you are not getting the exact amount for your exchange
as you see it in the current balance of the liquidity pool.
So if you really go back here, right at the beginning, you really would expect that for your 1000 USDC,
you would get half an ETH, 0.5 ETH. No, you're not getting it.
By investing in the liquidity pool, you manipulate with the investment, you manipulate with the investment exchange rate,
and you get an exchange rate that is based actually on your investment.
So for example, if you really would invest now 2000 USDC or even more, you would even get less compared to what you see here.
Okay, so, but there is a question.
Yes, can you hear me?
Yeah, yeah.
What would be the benefit of such a liquidity pool compared to just implementing an exchange that just matches orders in a smart contract, which would be possible, wouldn't it?
Yeah, you could also have an order book, you could also implement an order book. Yeah, you can do that.
Just another way of trading it. But on the other hand, remember that a smart contract is only a few lines of code.
So the more complex the smart contract is, and the more data the smart contract really stores, the more expensive the smart contract becomes.
Yeah, that makes sense.
Yeah, and then now you have, for example, you really have a large order book, this would mean that your order book would become part of the smart contract,
and suddenly your smart contract and each transaction becomes extremely expensive.
Now you could argue, okay, I just do the algorithm of the order book in the smart contract, and then I keep the order book separate.
Yeah, where would you store that? Hmm, in the IPFS? No, probably not a good idea in an external database, then you don't need a smart contract anymore.
No?
No.
Okay, good. So, so much about this. Let's see, what else do we have?
Another interesting concept is that lending and borrowing.
So, you can lend and borrow assets secured via smart contracts.
There are examples that make a dial compound if, in order to do that, there must be some security or collateral.
And we see this in an upcoming example. The main point here is if you want to borrow something, you need to provide a security for that.
Yeah, if you want to borrow money, the security is your house or the car, or you provide some other kind of valuables in order to get money if you want to borrow something.
Here, you don't offer your house as a security, you offer an amount of money that is 150% of what you actually borrow.
On the first hand, this sounds strange. So, if I want to borrow something, let's say if I want to borrow 10 euros, I have to provide a security of 15 euros.
Would that make sense? Because then I just can take the 15 euros to invest. But we see why this could make sense.
On the other hand, if the collateral falls too much in value, then the loan is automatically terminated.
And that leads already to the point that we say, okay, it seems that we borrow one cryptocurrency against another.
And that is exactly what happens. We see this in an upcoming example.
But you also see here is the stablecoin. So that's another DeFi concept, is that a cryptocurrency is packed to real-world assets, for example USD.
USD is packed against USD. So there we have a price stability and usability in everyday life.
So you buy 1000 USDC and this is worth 1000 USDC and you don't have to be afraid that when you sell it one week later on that the value has halved or maybe even doubled, whatever.
So there are several ones. And I think we discussed that.
So the whole issue is really when in an ideal world when you buy 10,000 USDC, that on the other hand the organization that provides you the USDC puts the 10,000 USD somewhere in a safe.
So they put them somewhere so that immediately when you would like to say, I would like to change my stablecoin back to USD that they take the money out of the safe and provide it to you.
So that's the point. And in the meantime we have, it all started with US dollars because they have a much weaker regulation.
But in the meantime you also can have Euro USDC which are even controlled by the BAFIN. So the BAFIN has the regulatory office in Germany.
So there you can start a Euro stablecoin company and then you have to follow their rules and then they control what you do and they somehow control that for every Euro you get you just don't invest it into any other thing so that you become bankrupt if I would like to get my Euros back.
So but what happens now is this lending and borrowing. That was the point actually. So lending is that you lend tokens and you are in interest and borrowing is that you borrow tokens and you deposit collateral.
And this is all done on smart contracts. And so for example the big example is that you as a user A, you lend 10,000 USDC and then user A lends you 10,000 USDC to user B.
And user B borrows then stable cons from it. So let's look at how this works. So we have the users.
We have a user who would like to get USDC. So your goal is that you would like to get USDC. So you would like to borrow USDC.
In order to do that you need to deposit a collateral. And you have for example 1.5 ETH in your wallet. But you would like to get 2,000 USDC in addition.
What you do is that you deposit the 1.5 as a collateral in the smart contract. So you provide 1.5 ETH which is worth at the moment 3,000 USDC.
And in exchange you get 2,000 USDC. So loan to value is 66%.
Maybe this is mind boggling because actually you have something that is worth 3,000 USDC and you get only 2,000 USDC for it.
Now if the ETH price now falls sharply the collateral uses value. So what you deposited is 1.5 ETH which is at the moment of the transaction worth 3,000 USDC.
You got 2,000 USDC now in your wallet. Now imagine that the value of ETH decreases.
Now here the assumption is that 1 ETH is 2,000 USDC. Let's imagine it falls down to 1,000 USDC.
So your collateral is suddenly worth only 1,500 USDC. But you borrowed 2,000 USDC. So your security or your collateral is less than what you actually got.
And this would lead to an automatic liquidation of your loan.
And even before that if the collateral value drops below 120% it's automatically liquidated.
So your deposit of 1.5 ETH if this falls below 120% of 3,000 USDC.
The 1.5 ETH is being sold on the market immediately. And this is now being done in order to save the overall borrowing scheme.
And I think you may ask why should I do that? Why should I go into this risk? Do you have any idea why users would deposit 1.5 ETH?
Which is even more than what you will get in order to get these 2,000 USDC. Do you have any idea why people should do that?
Or would do that? It happens. People do it. So there is a good reason for that. Do you have any idea?
No? No guess?
So the German term is FANTHAUS. Not sure what the English term for that is.
But in a FANTHAUS you go and you provide your watch, you provide your golden necklace or whatever and they pay you money out of it.
So let's say I have here, it's not a valuable watch but it's an Apple watch which is quite old but I would like to bring it to a FANTHAUS.
Let's say it's worth still about 100 EUR. So I wouldn't get 100 EUR for it. I only would get 50 EUR for it.
But people do that. Why do they do that? They need money. The point is really they just need money.
And they don't want to sell the watch to someone else. They could also probably sell the watch for 100 EUR to someone on the black market or whatever.
But they don't want to lose that. So they don't want to lose this particular thing.
And therefore they provide it as a security in order to get additional money.
And what a lot of people do is why people would actually do this particular exchange here. 1.5 for 2000 USDC is that they want to keep the 1.5 ETH and they want to borrow the 2000 USDC in order to pay something else.
Or they just need it for a short term notice.
And even more risky is what people do is they do the following. They believe that the one point that the ETH value will increase.
And then they borrow 2000 USDC for the 1.5 ETH and they invest this 2000 USDC immediately into ETH.
So they buy even more ETH for the 2000 USDC. And they hope, or they expect or whatever, that ETH will increase in money.
So the value of ETH increases. This means they still have the 1.5 ETH. They have additional ETH that they got here.
And let's say they say, well, ETH will increase in the next 10 days by 10%. So for the 2000 USDC they buy additional ETH.
After 10 days they sell the ETH and then they sell back this and then they even got more money.
So they're just somehow accelerating the whole thing. This is the positive effect. Imagine that the ETH value drops.
Imagine that the ETH value drops. This would mean, oh my, my, Tony Harris is somehow interacting with me.
And imagine that the ETH value drops. All these financial decisions.
So imagine that the ETH value drops. This would then mean that you lose somehow what you borrowed here and you will lose your deposit, which will end in a complete chaos.
So we still have then your interest rates. So you still have to pay interest rates.
So it's not that you just get this here for free. So it's also that you pay interest rates for the whole thing.
So this is just an ideal exchange. In addition, you have to calculate the interest rates for the whole thing.
So at the moment you get as a lender, you get 2% as a revenue and the borrower has to pay 4%.
And the rest is just the safety margin of the system. So it's the smart contract somehow that earns money.
The difference here is 2%. And you see some reasoning.
So the motivation is that the borrower really wants liquidity. Stablecoins in a short time is outselling the asset.
You don't want to sell the ETH. This is the example. The user has 10 ETH.
He believes that the price will rise, but he needs 2,000 USDC in short time to participate in another project.
So he sells the ETH and then he just doesn't need to sell his ETH.
Another thing is also, for example, if you have an image that you just bought ETH and you sell it,
then you have, let's say, you bought the ETH at a low value and the ETH price increased.
And if you need money now, then it's much better to borrow the USDC and not selling the ETH,
because if you would sell the ETH, you probably would have to pay taxes, because you got an increased value for ETH.
So you have to pay speculation taxes on that. And instead of doing that, in order to, instead of paying the taxes,
you calculate the risk of taking the loan, of borrowing the money, and therefore you just borrow it and you don't pay it.
So there are some kind of good reasons beyond just extreme speculation.
Okay, seems that this becomes clear, no further questions.
There's a question. Okay, go ahead.
Do you have to pay some sort of interest rates if you borrowed money?
Yeah, if you borrow the money, you have to pay 4%.
Okay, good. So there are some other kind of concepts. You can have derivatives or synthetic assets.
So for example, this is more or less tokenization, we discussed this already.
So you can have tokens or not something that, for example, token exchange for a stock or something like that.
So this is something that we discussed. And then we also have this whole kind of oracles.
We also discussed this when we discussed holy market and other kind of things.
Okay, so finally, the last slide is that of decentralized autonomous organizations.
You have a collectivity controlled organization without a central management.
For example, what we could do now is that we set up a DAO.
So we go to make a DAO and then we just create a DAO for us for the lecture.
And we say, okay, let's invest. Everybody can invest his crypto tokens.
Let's say ETH. It doesn't matter how much you invest.
Let's say one person invests one ETH, another person just invests 0.01 ETH, just a small amount.
We all invest that into our DAO. So we all put it into a pool.
And now one of us is making a big, cool proposal.
He says, well, I just heard that this kind of meme coin just issued by someone will increase in value in the next month by 100%.
And he convinces us that this is a good idea.
And then we do a voting. And then we vote. We vote on that particular proposal.
So one of us is then making this proposal. He says, okay, let's vote on investing into the Web3 meme coin.
And if we got enough votes, let's say the voting is 50% or 75% or whatever.
That is something we agree upon. And then we vote.
And then the voting can be based on the amount that everybody has invested.
It can also be based on one person, one vote or whatever.
That's all what you can control. And then as soon as this voting reaches the majority, so 50%, 75%, whatever we agree upon, then this transaction is taking place.
So then automatically the smart contract will issue the particular transaction.
So the smart contract will then take all the money or let's say 10% of all the money that we invested and buys meme coins for that.
And therefore we call this a decentralized autonomous organization.
So it's not completely autonomous. It's somehow autonomous in the sense that we can do some kind of voting and then the transaction automatically takes place.
We don't need a DAO for that. We could meet once a week and then we discuss where we would like to invest our money too.
But you could also do something like invest in such a DAO and you even don't participate in the different voting.
You just say, well, these are clever people and they always make the right decision.
Just give them a thousand euro and let's see what happens after one year.
Again, it's in the whole area of speculation. But this is what normally is being done with the DAO.
You could also use the DAO for example to make a decision on let's say you are in a cinema club and you would like to decide where shall we go next Friday into the cinema.
And then you just do this for voting and the vote then just comes, okay, we watch the new blockbuster or we watch any kind of cultural film or whatever.
And you can just use it as kind of voting. Then it becomes just a simple voting mechanism if there is not really a immediate action connected to that in the end.

Okay, so that's so much about the different DeFi concepts.

## Q&A about exam

ab 42:45

And now I'm open to your questions regarding this or any other kind of questions that you have regarding the action.
Okay, that's one question. Go ahead.

If you go back a few slides where there says that the lender, it was already there, the lender gets 2% or has to pay 2%.
I didn't get, I get that the borrower has to pay 4% in order to, because like to just amend for the lender, but I don't see why the lender has 2%.
This is what you get. You are the lender, you invested into this borrowing pool. You gave a thousand euros or a thousand USDC into this borrowing pool.
And if other people borrow from that, they have to pay 4% and you get 2% for the money that you put into the borrowing pool.
This is what you get as someone who's providing money to the pool. There must be some people who offer the money.
Okay, so the borrower pays 4% of interest of the lender or the lenders, only get 2% so aren't there then 2% missing?
No, the 2% get into the safety margin of the system. That for example increases the overall money in the pool.
So there's even more money in the pool in the end. Or this is being automatically sent to the person who is running the smart contract.
So you can have a smart contract. If it's you, you deploy the smart contract and you say, okay, the lenders get 2%, the borrower has to pay 4%.
So you implement this kind of security mechanism here. This kind of, if it drops below 120%, then it's being automatically sold.
So it seems like a very safe system. And the difference here, the 2% are being automatically transferred to your wallet.
And after some time you may become rich.
Okay, and after the automatic liquidation, if it gets below 120%, the person still needs to pay back the amount of money they landed, right?
No, because what happens here is that, let's say you borrowed 2,000 USDC but you provided 1.5 ETH.
If now the value of ETH drops below 125%, which is then, let's imagine 1.2 ETH.
So then automatically the 1.2 ETH are being sold for, let's say, 2,200 USDC.
Which means that we as the pool owners here, we get 2,200 USDC.
And we gave you only 2,000 USDC. So we made a profit of 200 USDC, which is then also with us.
And you as the borrower, sorry, you get your 2,000 USDC, that's it.
But you've lost your ETH. We took away your ETH. You won't get the ETH back.
On the other hand, if you would pay back the 2,000 USDC, so for example, you see, oh, hold on, there is something going wrong with the ETH exchange rate.
So it's dropping and dropping and dropping. Then you pay back the 2,000 USDC, you get your 1.5 ETH back.
And then you keep it in your wallet and you hope that it raises again in value.
But if you don't do that, we just sell the 1.5 ETH and then it's gone.
And the 120% are just to make sure that still we earn some money.
Because 120% and 20% they guarantee that we get more than the 2,000 USDC that we gave you.
We get a little bit more, let's say 1,000 USDC, 2,000 USDC, about 100 USDC or whatever.
So we are earning 100 USDC.
Is that clear?
Yes, thank you very much.
Okay, good. So any other questions that you have on the overall lecture regarding the exam or whatever?
I think you mentioned at the beginning of the first lecture that there would be another lecture about the legal side of blockchain and all this stuff.
Yeah, that's not happening.
No, no, that was given.
I have a question as well regarding the exam.
In the beginning of the lecture, you mentioned that the homework could or would become prerequisite, but I'm not sure.
And then I saw there was nothing announced.
So can we see the exam regardless of the homework status?
Yes, yes, yes, yes.
You can participate regarding the homework.
The homework was somehow, I think it was preparation for you in order to be more prepared for the exam.
But the participation in the exam does not depend on the homework.
Okay, another question?
Yeah, I still have a few questions on the Ethereum material.
Yeah.
So first of all, in the slide where the rewards and penalties for the attestors and proposers and aggregators are explained,
it says there takes place punishment for double proposal flash at this station.
So double proposal is probably if more than one block is proposed.
But I have, I think 512 attestors.
How would double attestation work?
That's one of the attestors.
Oh, it would that you probably, I mean, what the tester does is that he says,
well, the block has been produced correctly, that he tries to do that two times.
So that's what I guess from that.
But if he says the block was correct on the same block, because he is the tester for one block or for one epoch, I think.
And then he is again, no attestor again.
If he says for the same block, okay, it's correct twice.
Where's the problem?
Maybe just, oh, I have to guess.
I don't know.
I have to check.
Now, imagine based on this, it wouldn't become a question in the exam.
Okay, thank you.
I have another question regarding the format of the exam.
So I would assume it's more about understanding the mechanisms, so open end questions,
or will we have multiple choice or what would be the format?
Both. It will be both.
And you're right. It's more about understanding the questions.
So it's not about what's the current price of the, something you can look up.
It's about the general understanding, and it will be a mixture of open-ended questions and open-ended questions.
And let's say there's someone else who would like to join.
Open-ended questions and multiple choice.
It's, even if there are open-ended questions, you don't need to write long texts.
So there will be nothing like, please explain in detail the proof of work.
Such that you have to explain how the proof of work in general works, and if this and that, and for example,
it could be more specific on some kind of aspects of the proof of work.
So for example, it could be something, why is proof of work time-consuming?
And then you could answer the proof of work time-consuming, or you wouldn't need to say that.
You just say that because it's based on the solving of a crypto puzzle, which requires computing time,
and that cannot be algorithmically solved, something like that.
Yeah, thank you.
Professor, sorry.
I just wanted to ask, I'm still not able to register for the exam.
Yes, because you are physics, yeah?
Yes, exactly.
Yes, what I suggest is that, because I think the whole lecture is not registered with the physics study,
what I will do is I will provide you a written statement about passing the exam.
Okay, thank you.
Yeah, so I will provide you a written statement with the grade and the credits and so on,
and then you can go to your own local ZPA, to your own Prüfungsauschuss, and provide that and get it accepted.
Okay, perfect. Thank you.
And we also had another student who is currently still in Bachelor's studies.
That's the same case. I also will provide a written statement that the student can then use when he started his Master's studies to provide this as a statement.
Yeah, that was me.
Thank you, Professor.
There's another hand.
Yes, are there going to be smart contracts, programming in the exam?
No.
No, okay. And are there going to be some like, tasks where you have to calculate something?
No, no, no, no. I mean, I thought about, would it be another example that we do some kind of liquidity pool,
and I give you some transactions, and then you have to calculate that.
No, no, there is nothing about this kind of calculation.
But I think it would be a nice question to ask you.
Let's imagine that you have the following liquidity pool in the following ratio, and then you invest that.
Would you get this or would you get that?
So just to check if you have understood that you are not getting what the current exchange rate is,
but that your trades will manipulate the exchange.
You got what I mean?
Yeah, yeah.
So you're not getting 0.5, you're just getting 0.48.
Another question?
Yes, so again, about Ethereum and also that's our identity.
Yeah.
I have two questions.
The beacon chain is part of the Ethereum chain, but from looking through the slides again,
I did not recall the beacon chain.
Is it like the chain for the proposers and the testers?
Yes, yes, yes.
It's the chain that is somehow managing the overall proof of stake, while the Ethereum chain is the chain
where you really have all the data, the smart contracts and things like that.
All right.
And then about self sovereign identity.
It says that, for example, one is able to prove that they are over 18 years old without providing the birth date.
Yes.
But in order to, I don't know, get the attribute over 18 added to their wallet, they have to,
I mean, like they have to leak their birth date at least to the issuer, right?
Yes.
Yes.
Okay.
Yeah.
That's the case.
So what's happening there is the following.
If you get a certificate about your age, or if you get a, if you just digitize your personal passport,
or if we digitize your driving license, then there is an issuer of this particular credential.
And this particular issuer, he has to check the data.
So you provide him your driving license, your personal house wise, you provide it to him.
He then certifies this date in your credential, which you have in your wallet.
And if you now go to a shop and you would like to buy some alcohol, normally they ask you for your passport.
And then the point is that they look at your passport and they see a lot of information.
They see your exact birth date.
They see your birth place and all the rest because you provide them the full amount of information with the whole SSI concept.
If they send you a request, is this person above 18?
So they would like to have a proof.
And this is being evaluated in your local wallet and being sent back to the shop owner.
And the shop owner then gets the information.
Yes.
That person is above 18 and that is being certified by the police station XYZ or by the Einwohner Melderamt in Aachen or whatever.
And there you see the point.
You have to trust the issuer of this particular certificate.
So I could provide you a certificate that states that you are above 18.
And then the shop owner would get the certificate.
Yes, this person is above 18 and this is being certified by Wolfgang Prinz.
And then he may trust me or not.
For example, if you apply that to a particular business environment, we would all have our particular passports here for Fraunhofer.
There would be one central authority at Fraunhofer that is currently issuing our passports with some kind of additional information on top of it.
And there it would be okay.
If someone checks are you a Fraunhofer member or are you a member of Fraunhofer fit?
Yes, no.
That would be certified by the Fraunhofer agency.
For this we don't need any Einwohner Melderamt or police station.
That's the whole principle.
This is actually currently being under development in Germany.
There are currently just this week there was a big call for proposals for companies to develop this particular infrastructure, to develop this kind of ecosystem, such that central providers can provide these certificates based on a high official level
and maybe also based on a small official level because it may also be that you get a small certificate today that you were travelling by bus.
And this certifies that the bus was late and therefore you don't get any punishment with coming back late to the lecture.
And I get that it's being certified by the bus driver, it's okay.
Any other questions?
So overall with the exam what you get is you will get a paper document that contains all the different questions.
You don't need to bring any own paper.
Now it will all be provided by us.
Is the video from last week somewhere available?
Yes, I will upload this together with the video from today.
I just still have to download it.
I saw there was also a transcript.
I have not yet really checked if the transcript is really good, if the transcript is somehow usable, I also provide that.
Okay, good.
So if there are no other questions, then thanks for participation in the lecture.
I will upload the remaining material and then we see ourselves at the exam date.
Okay, bye bye then.
Bye bye bye, thank you.
Bye, thank you.
Thank you.