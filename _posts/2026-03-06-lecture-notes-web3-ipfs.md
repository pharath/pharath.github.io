---
title: "Web3 and Distributed Ledger Technology - IPFS"
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

Content identifier and this is how it looks like and that's how it's being constructed.
Knowing that you don't start programming on the IP and as I've just done programming to develop your own content identifier,
you don't really need to deal with that.
This is all done by the system itself, but this is...
I understand how this is being constructed and you could also use this kind of algorithm as you would like to address any other kind of information,
which probably do not start on the IPFS but that you would like to point your friends to that you start on the IPFS.
Okay, so, guys, the first thing. The content identifier.
The next thing is now how do we actually store the data on the IPFS?
And that's also quite interesting.
So what we need to do is that we need to store the data in the IPFS and we discussed already that we need to store different kinds of data.
So we need to store JPEG images, video images and P4, but we also need to store, for example, the whole directory where we store our sources.
So imagine you're running a nice program where you have different sub files and different kind of HTML pages included and we also need to store that.
And for that we need to find a way how to represent this actually in the IPFS.
So we have already this way how to serialize that.
But the next thing is that we say, okay, when we have a very, very large data file, let's say a big, big image, your sources, or any kind of movie or whatever,
we say that we chunk it into blocks, which means we are not storing a big data file as a single block in the IPFS.
So the protocol says large files need to be chunked into blocks.
We see later on why this makes sense.
What we then need to do is that we say, okay, we have, for example, one megabyte of image and we chunk it into 100k blocks, then we need to generate a CID for each block.
So that's what we need to do.
So each block then has its own content identifier.
And the files that we have been chunking then in addition being organized in a tree.
So this one megabyte file that we chunk, for example, into 10 blocks is then a tree with 10 leaves.
That's the way how we do that.
And the properties of this Merkle duck are then that the CID of the parent node depends on the CID of the child node.
Similar to the Merkle tree that we discussed when we organized the transactions for the Bitcoin blocks.
And if we change then the leaf content, all nodes from the leaf up to the root change also the CID.
And we can also say that if we have the content identifier of two Merkle trees, if both are equal, then the trees are equal.
That's the case because the parent CID depends on the leaf CID.
And therefore if we have the root CID of two Merkle trees, if they are equal, then the content is also equal.
And also with the root CID, we can retrieve the whole tree.
So if we have the root CID, we can retrieve the whole file, which is necessary because when we chunk it, then we need to make sure that in the end when we just have the top node, we need to be able to get it.
Okay, so that's the way how this works.
And what you see here is then that there are some kind of explorers also and you see this here in this case that we have a particular file.
And this has been chunked into two different sub nodes.
So we have the file, that's the root node for the file.
And here are the two sub nodes for this file.
So actually this has been chunked into these two different sub files.
So that's how the way how we store information in the interplanetary file system.
So we don't put the one megabyte file as a single chunk onto the file system.
We chunk it into sub blocks, which are then organized in a tree.
So the question is now, and maybe you were wondering why do we do this?
Do we have any kind of information why we do that?
Probably on the one hand it must be much easier if we just store it in a single piece.
Why do we do all this hassle of chunking it and building trees out of it and things like that?
Do we have any idea why we do that?
Yeah, go ahead.
You just need to unmute yourself and then...
Well, I'm not sure, but given the decentralized way of how we store things,
it may be easier to store one chunk at one server and another chunk at another server
and don't block an entire server by, I don't know, storing a terabyte project.
Yeah, that's a good idea.
So we have a lot of parallelization then, which improves it.
So that's one good argument.
You could say that when you retrieve a one megabyte or one terabyte file,
that you actually can retrieve it not just from a single source,
but you can open up 10 connections to 10 different servers
and then you have a parallel retrieval of the content.
That's a good argument.
There's another argument for it. So we have one.
There was another argument.
Someone has an idea.
Maybe I misunderstood something, but as I understand,
the files can go from one server to another if another server wants to retrieve those files from the system.
But why would I, for example, want to retrieve a few blocks and not the whole file?
You refer to the explanation that I just gave.
Yeah, well, the reasons for the following.
If you retrieve this from a, if you retrieve it just from one server,
you completely depend on the bandwidth to this particular server.
And in the end, you just do this then sequentially more or less.
But if you can retrieve 10 parts or 10 different parts of the same file from 10 different locations,
you can parallelize that.
And then you may get it in one tenth of the time.
You wouldn't get it in one tenth of the time,
but maybe you get it, for example, in half of the time.
They're just retrieving the whole block from one system.
I have a question on this.
But if, like, okay, one server is down, then that, like, if you paste it in different servers,
and it's down and then you lose that piece, so to say, right?
And then you can't retrieve it at all.
That's also the case.
Yeah, definitely.
If you have split up your file into 10 different chunks,
and these 10 different chunks are located on 10 different servers,
and one of them is down, and you don't have any replication of that,
then you wouldn't get your whole file.
That's true.
But we are thinking here about, somehow, about an ideal world
where all the different servers are up and running.
Which is not always the case.
But it could happen.
What I would like to show you to answer the question is this.
I just point, you know?
I just retrieved that from the sunrise image.
And this gives an answer why the, it gives another question.
So the first question was that we somehow can distribute it over to different files
so that we may get a faster reply of that.
The other reason why we chunk the information into different paths is that
we may reuse the same information.
Because if we have a file that contains information that has already been stored
in our distributed file system several times,
let's say a very often being used meme, an image, a standard text or whatever,
then we can refer to this particular chunk.
So imagine that you have, you are doing a lot of software development
and with software development you always include the same library within your sources.
And this would mean that you have the library already somewhere in the IPFS.
And you only then need to link to this particular library.
You don't need to store this within your large content as a single plot.
And that saves this storage.
And by having them multiple references to that,
which would also mean that all the storage system files are there,
maybe even retrieved on different servers also increases the redundancy of that.
But there's a question.
Is this done automatically or do one have to manage that?
No, this is done automatically.
I will later on show you the IPFS desktop client and then we see how this is done completely automatically.
Okay, so we have now two things.
We know how to address files in the IPFS.
And we also know how to store them in this independent error file system.
And now we get to content routing.
And content routing is now quite a complicated thing.
And I mentioned that already at the beginning.
There is a link to a YouTube video of someone who explains it very nicely,
which I can only recommend.
And I just recommend also to listen to that and I will try to give you an introduction on how that works.
Normally I use the whiteboard for that, which is unfortunately impossible now,
but I hope I can get you somehow an idea on how this particular thing works.
The actually challenge that we now have with our IPFS is,
after we know how we do the addressing with the content identified,
and after we know that we chunk files into different pieces,
then do we store them?
And how do we find them in the end?
So think about the World Wide Web.
When I give you a web address, rwth.de slash something, then based on the DNS,
this name is being resolved into an IP address and based on the IP address and the DNS,
people know where the server is located and then they're just forward your request to this particular server.
That's done in the Internet with the HTTP protocol works fine.
With the IPFS we have the challenge that so far the addressing of content is just more on SDH.
There is no information where to find it.
How do we find the server that is actually storing that information?
There is no server address within our content identifier.
How can we find the server?
And that's the challenge and that is now being solved with the so-called Catermilla distributed hash table algorithm,
which we discussed there are other kinds of gossip based routing,
which we will not discuss here, just for naming it.
But that's the solution now, how we find the information within the IPFS.
So the basic architecture is the following.
We have a lot of different servers running, PIRs, in our context here.
And each PIR has a unique ID, which is public key.
Somehow it's public key, see just as a name or see it as an address within the IPFS.
And this PIR is discoverable.
So you can ask PIR, are you alive or not alive?
There are some PIRs which act somehow as some kind of first addresses where you go.
And I saw some kind of anchors and then all the other PIRs that are around.
We see this when we look at the desktop application.
And it must be reachable.
So and it can also be reachable by different protocols because we have this multi-address protocols.
So if it's not reachable, then we can't get the information that is being stored on the PIR.
You mentioned that already.
We have some kind of encrypted communication.
The information that we transfer between the different PIRs is being encrypted.
And the important thing is that the content is not replicated automatically.
And here we have the big, big difference between the PIR to PIR systems that we know from years ago
where we used PIR to PIR file systems to share music files.
So when I was younger at your age, so there was not something like Spotify or whatever.
What we did was actually that you could share music with a PIR to PIR file system,
which was illegal because there was copyrights of music.
And the big danger was the following.
That when you participate in this network, just by participating in the network,
your note that you are running on your PC was automatically providing information to other nodes without your knowledge.
And in particular, this note was also retrieving information from other nodes without your knowledge.
So after one hour of having yourself up, it was already heavily providing copyrighted information to other nodes.
And if this was detected, you assumed and you had to pay a fee.
So this happened for several years.
People use several tricks on that to avoid that.
But this is not the case in the IPFS.
In the IPFS, you don't need to be afraid that your note is suddenly providing information to the outside world that you have never retrieved.
So there is no danger that you have suddenly you're providing copyrighted information,
photographic information or whatever can happen, unless you downloaded it yourself.
So that's the case. As soon as you access a file on the IPFS, then it's also being stored on your note.
So then you also become a provider unless you delete it from your note.
But that's an important thing, which makes it also very important for using that system for companies.
Now, for example, for Fraunhofer, I would never be allowed to run such a note if there is that danger that suddenly we provide any kind of illegal information at this system, such as that we become a content provider for illegal information.
Now, the next thing is now that point is that how do we store information about what is being stored in the IPFS.
I mentioned already, imagine that we need that I upload a file to our system, to the IPFS.
How do other people know about the information that I have uploaded there and how do they find it?
So the scenario is the following, I upload a file to the IPFS. It's being stored just on my server.
And then I provide you the CID to retrieve the file. How does your local server know to find my server where the location where the information is actually being stored?
And this happens by the following. As soon as I upload a file to the local server, my server informs other nodes and it has contact to about this new information.
So what it does, it actually tells that my neighbor nodes that I have uploaded information. But it's not telling everybody about that.
It's telling only those nodes about this information that are closest to the information itself.
So this is a little bit mind boggling. What does closeness mean here?
Closeness does not mean that it stores this information at the nodes which have the shortest geographical distance to me.
So it's not storing the information. For example, imagine that my neighbor just across the office here has also an IPFS running.
It does not mean that it's informing this node about the fact that I have uploaded, let's say, these PowerPoint slides to the IPFS.
It informs the nodes that are closest to the content identifier of these PowerPoint slides.
And this is being computed, the closeness is being computed by performing the XOR operation.
So what we do is actually that we do an XOR operation of the content identifier of the data and the address of the nodes.
And those where the XOR operation of these two addresses, the content identifier and the identifier of the node becomes the shortest.
They get the information that they can find, this information on my node, which is then addressed by the IP address.
So what we actually do is, and you see this here, and there you see an image of the Cademlia explained YouTube video, and you will find this also then in the slides.
It does a very nice job in explaining that from the very beginning on. So the video is about half an hour long and I really recommend to look this up also.
What you see here is in a very, very simple example, they just built the namespace of four bits to simplify it.
And what you see here is that we store somehow information. So this is information, the blue dots are information.
So the blue dots are information and the other dots here, this is a computer.
And what we do actually is that we place both the information as well as the nodes into this namespace.
So for example, if I upload the PowerPoint slides here, then the PowerPoint slides, they have a content identifier.
In this case, it's just the four bits content identifier 0 1 0 0.
And the information about that we store at the closest nodes, which have to the closest nodes.
And the closest nodes is n5, because the difference between 0 1 0 0 and 0 1 0 1 is 1.
So we store the information about our PowerPoint slides, we store them here at that particular node.
Plus some other eight other nodes. So we would also store it here and there, for example.
Just to have some kind of redundancy. You see another example that the distance between that node and that node is 1, so it's 8.
So it's 1 0 0 0 and four digits, so it's 8.
It's digital numbers.
So the actual way how this works is that we upload the information.
And then we inform the nearest nodes based on the address that we have uploaded this information.
So if you now want to find this information, you note, so I provide you now the content identifier.
And the content, your server takes the content identifier and looks up his own table, which is the closest node to this content identifier.
So he builds the XOR operation of the content identifier and the nodes that the client knows about.
So then he's asking the closest one, do you know more about this particular data?
And if this node does not know more about it, again, it will ask the nodes in its own system, in its own cache, who is the closest one to this content identifier.
And this in general takes two to three operations and then normally a node is found that knows about my server.
So it knows the IP address of my server and then it returns the IP address of my server to your node.
And then you can do this by a peer-to-peer protocol to retrieve the particular information.
Question, yeah. Go ahead.
So when I'm uploading information to my server, I'm telling the nodes that I already know and that are the closest to that information hash that I have this because otherwise I would need to know every node address, right?
Exactly. When you start a server, then you only see some nodes.
And I think the easiest way is now that we have to look at this desktop because so much theoretical information now we just play around a little bit with that.
So what you see here is, and move it over into this particular desk.
Well, now you should see my IPFS desktop.
Actually, you can download an IPFS desktop. Just look for IPFS desktop.
Let me see. I still have the web page open because I had to retrieve the latest version recently.
I am here. Let's see what page where I downloaded it.
So you see github.com, IPFS, IPFS desktop releases.
And the count release here is the version 0.47 and there you can download the different things.
And the easiest way to download it is really here to use here this Windows file.
Now download it, double-click on it and then you can install it.
So I just want to do is just copy it. So if you like, go and put it in the chat.
Here's the link. And then for example, if you like, you can just retrieve and download it on your laptop and then you are part of the IPFS.
If it's running. No, sorry.
And this is now the desktop. So we can see what is the current status.
So there are currently 28 megabytes on data that is being hosted on my server.
We detected 10 other nodes in the IPFS. That's my PID.
So you see it looks very much like content identifier. And my agent is Kupo. That's the particular.
And you see my content was what I did. And you see currently nothing is going on here.
So at the moment I'm not providing any kind of information to anybody and also I'm not retrieving any information.
So let's see, we can look what do we see here. There we see all the addresses of the different nodes that are being connected.
So then I can have a look at the different files.
And you know the files that I have currently uploaded. So you know the files that I uploaded.
So that's a 3D printer file that I upload just as a test. And what I can do is that I can also just remove it.
Okay.
Okay, so this means that I'm no longer providing that information to the IPFS.
And if I would have provided you a link to that one, it would probably no longer be available for you unless someone else downloaded it and put it in his own IPFS.
So we can have a look at the files here. Click on it.
And then you see that's an introduction to a book that we just published recently about the Web 3.
And I uploaded it here. Actually, it's a preprint of the book and I put it here on the IPFS.
So that's it. And look at it and here we have another chapter that I uploaded.
So that's the way how I put it on the IPFS. And you see here that's the...
What I do is that I can take the link.
So I copy it and I put the link here into the... into our chat.
So you may now use this particular link. Click on it and maybe even your browser may be able to open it.
But it may also be possible if you like. You can just take the CID like that and you can paste this CID.
If you have a solid drop, download the IPFS desktop. Just put it here into this particular field and then you can retrieve it.
And then your server or your client, your desktop would retrieve it from my client.
Maybe someone tries it out and then we see if this happens. Let's see here at the status here.
Ah, yeah. There is something from traffic coming in. Is somebody already trying? Do that?
Yeah, I just downloaded it.
Okay, perfect. So this seems to work.
Right. So what else can we do? We can never look at the other files.
So you can explore the Merkel tree, the Merkel forest. But we can also go back to this one here.
And if we take here, and I think you can do the same now in parallel when you have downloaded this, you can say, let's investigate it.
And what you see now is that you see the image that we saw already on the slides, that the system automatically separated my file into two sub files.
So it was chunked into two different blocks. And each block has its own content identifier and we have the content identifier of the anchor, of the root.
So if we have different files, that's something larger. Let's see.
Ah, you see, and this has already been chunked into six different files.
If you like, you can also upload any kind of information now in your system and then you can share the link here in our chat and then another bit can share it.
And that's the way of sharing. So it's a very nice way of sharing.
On the other hand, it's really you don't need a third party for doing it. You're not uploading it to Dropbox or somewhere else.
You just provide it locally. And that's the way how it works.
We can also have a look here at the peers. And then you see that I'm currently being connected to 16 different peers.
So for example, in the United Kingdom or wherever. And if I would like to retrieve now some kind of information, then my system would take the CID, send it to the nearest one based on the XOR operation, which maybe somewhere in Mexico has nothing to do with the geographic topology.
And ask this one, do you know which kind of server is currently hosting that? And maybe it's then even going back to your local client to retrieve it.
But it's really the case that the server in Mexico is actually the storing this information because it's closest to the information.
Yeah, that's how this IPFS client works here. You see currently we have 709 peers, 751 peers that are currently all over the world.
And there are some companies that really operate larger servers to provide some kind of redundant information.
What else can I do? I can also do this kind of pinning. Yeah, unheavened.
So this means pinning. What I can do now is that I would like to pin this information which means if I do it now.
This kind of client that is currently running here on my desktop, it's using, I can tell it, please use 10 GB of my hard disk.
But if I now browse a lot in the IPFS, it will download a lot of information.
And after some time, the 10 GB are full, then it will start to do some kind of garbage collector.
And then it will remove files from my server, which are probably stored somewhere else.
But it will not remove those that I have pinned.
So with pinning, I somehow tell the system these are important files and please do not remove them when you do any kind of garbage collector.
Okay, did someone... Oh, yeah, there's a question. Okay, go ahead.
Yeah, basically I have multiple questions. So firstly, when we just launched the system, I mentioned that we don't know any sort of peers.
So is there like some separate mechanism so we can identify our peers or some other servers?
Yes, yes. So I would lie when I would say I really know how it works, but I think what they do is some kind of broadcast.
They do a broadcast into the internet to find other kind of peers.
And then when they find a peer, then this peer is telling them, look, I also have this other kind of peer.
So it's some kind of bootstrapping.
Okay.
Then the question, like if the address of our server somehow depends on the content which we store, which we provide,
so should our address then change when we upload new content or move new content?
No, no, the address of your server is just being determined when you upload your server.
So it's actually a public key, a private key that's being generated, which is just generated arbitrarily,
and it's absolutely independent of the content that you store.
But I mean, how does this system work then if we compare using the XOR operation and how does it work then?
It does the following. Let's say if we just take the example that your server has been started and it became the address number 4711.
Take it very simple.
So your server has now the number 4711.
And I'm uploading information that has the content identifier 4712.
And this is very close.
And then it will store the information that I have uploaded, that I provide a file with the content identifier 4712 at your server,
because your server is very close to it.
But this is just by accident, just very completely arbitrary.
You by accident got the address 4711 and I uploaded content with the hash 4712.
And it's just completely random that you then store this information.
But it's not just that you, it will also store the information at 4715 and let's say 4600 or so.
Okay.
And that is really the point.
When you first look at this cadamlia algorithm, you really have to separate between where is the data being stored and that has nothing to do with the nearest and closeness.
You store the data at the servers that have uploaded it more or less.
The information about where it's being stored, that is being stored at the closest content closest server.
But they don't store the information itself.
My file 4712 is not being stored at your server, just the information that you can find it at my server is being stored at your server.
Okay.
So, good.
So we are back on the slides.
Okay, so this is also being very nicely stored here in this video that I can recommend.
And then, yeah, I think what I just explained is also explained here in this particular system and particular slides.
So the provision example, I provide data to the IPFS.
So I compute the CD of the data.
Coming.
Okay.
So I compute the data.
Okay, so.
So I just compute the CD of the data 4712.
Then I look up in my connection table that it nearest to my data.
This would be 4711.
So it would be your server.
And then I ask your node to store the lookup information from my information.
And if you have now peers that are closer, let's say you are 4711, my data is 4800.
And then at the moment you are the closest to me.
So I send it to you 4711.
And then in your lookup table, you have a peer that has the number 4800 and one.
And then you say, okay, I know a much closer one.
I send it to this one.
And that's the way how it works.
So uploading means telling all the closest nodes about where to find the information.
And when you look up the information, then I asked my neighbors for the content.
So I would ask you now if I look up something that has the address 4700.
I know you as 4711.
I would like to ask you, then do you know this?
And if you don't know that, you look up the nearest node to this one.
And then this node provides me the information.
That's the IP address to use BitSwap to retrieve the actual content.
So in summary, actually the content routing is based on this cadamilla DHT, which existed already beforehand.
So cadamilla was already available before IPFS has been invented and using the BitSwap protocol to retrieve the data.
So it's a bit bigger protocol.
The data is distributed by PRIDs that are close to the key in the hash table row.
And every node maintains this information.
And in general, about 20 buckets of this address prefixes.
And with this 20 buckets, the experience has shown that with up to two or three hops, we actually find the provider of the data.
And what you do is you really ask the closest ones to ask the nodes that are closest to your data and then actually you will find them.
To sum up, what you should remember actually is the whole thing about the CAD, is content-based addressing.
These are immutable links, unique identification of self-certification of data.
So it's a sentence with a lot of interesting things. Immutable links.
So when you modify the link, you wouldn't get the data anymore.
Link and data are married to each other.
You have unique identification and you have a self-certification of the data.
Because when you retrieve the data and it doesn't belong actually then to the hash, then you can say, hey, somebody was cheating me.
And the file system, the interplanetary link nodes defines the standard to represent the data structure as mercantiles.
And the mercantiles is what you saw there in the desktop.
You saw that the file that I've uploaded is really being split into 10 different chucks.
The distribution is, it's really a community system now.
As soon as you start running your server, you become part of the community and providing this kind of information.
And it's also being, as we discussed at the beginning, so you can, as more servers we have, the more connections you have really to retrieve the file.
Which makes it faster, the whole system.
And we have some kind of duplication benefits.
So the same data results in the same CID.
As you see, I've uploaded here the introduction of this book that we published.
If someone else starts publishing this book, the same file, it will maybe stored on its server.
But it's also being on my server.
And we somehow don't de-deplicate the whole thing.
Even if he then removes his server from the net, it's still on my server.
So on the same hand, we have redundancy.
The issues are definitely permanent storage.
It may not be accessible every time and permanently.
Some of you mentioned that already at the beginning.
What happens?
Server is down.
Server down is down.
You don't get the information anymore.
For that you can use some kind of pinning to avoid at least that it's being removed by the garbage collections.
Or there are two developments, Filecoin.
It's the RBNB for storage.
You can run a Filecoin server on your own server.
And by that you somehow rent this space to other people.
But you have to do it in a reliable way.
And if you do that, you get some kind of crypto.
You get the Filecoin.
So the Filecoin is the currency to pay you for providing storage in the Filecoin network.
And if you don't do it in the end, or if your server is not reliable, you wouldn't earn any money.
So it's very nice to compare it with the RBNB for storage.
And RWEF is more a commercial operated system where you pay for uploading information.
And they guarantee that the information remains persistent on the server.
Yeah, there are some links which I can reprimand and which I can recommend.
There are tutorials here on the IPFS and there are some other interesting links to deepen the whole thing.
Okay, with this I'm through the topic.
We still have a little bit of time for questions or clarifications or whatever.
Do we have any questions?
Yeah, okay.
So in the example where you said that someone has the address 4711, your content has the address 4800.
And then you say the 4711 client that here this is the closest this is the data I have.
And then they say, okay, I know someone at 4800 and one.
So I tell them that there is the data.
Do the 4711 people still keep your address for that information?
Yes, they do that as long as I mentioned this K20 buckets.
So they store a certain amount of lookup information and as soon as they are still, if the information is still among the range of the closest information, they keep it.
Otherwise they apply a garbage collection and they remove it because then they say I'm too far away from this information.
I have already information about much closer notes.
And then I remove this information because this is no longer really relevant for me.
And the size of the buckets is then if you have 20 buckets, it's 120th of 256 or no, it's just 20, 20, 20, I think it's 20.
But yeah, probably need to look this up.
But they normally store 20 other notes about this.
But you got me there.
Because from how I understood it, if you have 20 buckets of the whole address space, let's call it that way, the size of each bucket is 120th of the entire address space.
Yeah, and I think I would need to look this up. Let's discuss this next time. I check this about this bucket size.
Yeah, I think this is also being addressed in the video. He talks about the bucket size also.
Any other questions?
Okay, if this is not the case, I will upload the slides.
Feel free to play around with the IPFS desktop. If you'd like to share information on your friends or to publish any kind of other things that you would like to share.
As I said, we have a institute meeting next Wednesday.
I would be able to come to Aachen. Would it be okay if we have the lecture then again as somehow a Q&A session next Friday morning again?
Yeah, I mean no problem.
Okay, if this is being cancelled, which I don't think, then I tell you, but at least on Tuesday I can inform you about the situation.
And then I will just publish the link to a video link and that would then be the last lecture, I think.
And then we can just have a Q&A session and I probably can just go through the slides and you can answer your questions.
You can ask your questions. I hopefully can answer them and I will point out some things that probably show up in the exam.
Okay, then thank you. Okay, then I stop the recording. That's important. I need to stop the recording.
Okay, good. So then thanks for joining on Friday morning. I wish you a nice weekend and then see you next week. Bye bye then.
Thank you.
Bye.
