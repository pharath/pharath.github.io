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

The description will be in the end.
You can still see the shared screen.
Hopefully.
Okay.
Objections.
Good.
So, then we can start.
Okay.
Yeah.
Welcome to this lecture on the Interplanetary Files system.
Thanks a lot for the flexibility that we do this online today.
Since I had this meeting last Wednesday, we had to defend a proposal that we submitted
in the area, also in the area of blockchain, augmented reality in the end.
But the good thing is that we got the project.
So we hopefully can start next year with this project that we get the funding, which would
also mean that we probably can hire some students.
So, but today we would like to talk about the Interplanetary Files system.
And I mentioned this already several times when we talked about NFTs and when we talked
about other systems where we would like to store information, where we must be sure that
the information that we get from the file system is really what we were asking for,
not just what we were addressing.
And this is what the Interplanetary Files system solves.
And the nice thing is it has a very nice name, the Interplanetary Files system, but it also
uses some very interesting IT concepts.
So what we're going to discuss there today are different aspects.
We talk about the content identifiers.
We talk about file structures.
And then we discuss the routing of content within this distributed file system.
And then we sum up.
Good.
So what about the Interplanetary Files system?
I took up two different definitions here.
One is from Wikipedia and one is from the IPFS document, which I can heavily recommend.
I mean, there's a lot of documentation about the IPFS.
They even have some kind of tutorials there.
And it's quite nice to look at this.
And the thing what Wikipedia says is, which is probably really already the most important
thing, Wikipedia says that it's a protocol for peer-to-peer networks for storing and
sharing data in a distributed file system.
So we have this distributed file system.
And the most important thing here is, let me see if I can highlight this here, is that
it uses this kind of content addressing, oops, that's not nice, to connect all the different
devices.
So it's really content addressing.
Content addressing based on an address.
You see later on what this means.
And I think we discussed this already a little bit.
In what we see at the IPFS website, it says that it's a modular suite of protocols for
organizing and transferring data designed from the ground up with the principle of content
addressing again in a peer-to-peer network.
It's open source.
And there are different multiple implementations of the IPFS, which is quite often you have
this standard and then you have different implementations of the same standard.
And its main case is really publishing data in a decentralized fashion.
And you may say, why do we have IPFS?
We have already HTTP.
We have the World Wide Web, which main use case is also publishing data.
But this is publishing data more or less in a centralized way.
And what we do here is to publish the data in a decentralized fashion.
So let's see.
What are the characteristics of the IPFS?
First, it's really decentralization and it's content addressing.
So we address the content based on its content, not the address.
We can also do version control.
It's based on a peer-to-peer network.
It provides us caching and a local storage.
And you will see this later on.
If you have a short demonstration of an IPFS client.
And then we have some kind of resilience and redundancy.
So in the IPFS, we store information not just at one node.
We can store information at several nodes.
And this very much depends on how popular the information is.
So the more people really retrieve the information, the more redundancy we have.
If we just store information in a single node, what we do later on in a short demo, there
is not much redundancy because I just store it on my node.
But if someone else retrieves it after the end, then it's being stored at several nodes.
And it's really built on a community NACO system.
So it's really built upon the idea that there are a lot of people that run IPFS servers,
which then provide overall the IPFS in general.
And if we look at the basic principles of the IPFS, you will find one.
The first term is this about causal linking, which means that we have a linking of objects
based on the existence of this object.
So we can only link an object that exists already.
This is what we understand by causal linking.
So objects are linked after each other.
The linked object must be in existence before it can be linked, which means that if I upload
a document, then this document can only be referred after I have uploaded it, obviously.
So I cannot just point a link to something that is not just in existence.
This very much refers also to the blockchain.
You cannot refer to a block before it's been in existence.
So you can only refer to blocks that exist already in the past.
You cannot have a future link to something that will become existing in the next time.
That's, for example, a big difference between the HTTP protocol and the World Wide Web.
I could now give you an address of a web page, and this web page might not exist.
In the IPFS, that's not possible.
And I also could not give you a link to a blockchain block that does not yet exist, because the
link is actually the hash of the block.
There you see where the story goes.
And you can find this file anywhere.
You don't need to have a central authority to prove the file.
Anyone can surf the file, which means that if I give you the address or the link to a
file in the IPFS, you don't need to take care of that only trusted providers own this file
or provide this file to you.
Anybody can provide this file to you, and you don't need to trust this particular provider.
The trust is in the case that the file is being addressed by its hash.
And if you get the file, you can always check the hash of the file to see that the delivered
file is actually what you are asking for.
This is not the case in the World Wide Web.
If I point you to something, then the owner of this web server can always change the
content, and I cannot prove that the content is still correct, because you just address
a location.
The big advantage is that you can check the content that came before another content.
For example, that's quite interesting.
For example, if you say, I'm for my analysis of my scientific paper, I'm using the following
data, and this data is being addressed by this particular link, by this particular causal
link, then this data must exist.
You cannot just say, I'm basing this on the content with the name ABC.
This content must exist.
And it also provides you the information that the other content was before the second content
that is linking.
And for example, in patent discussions or things like that, this can be quite relevant.
The other thing is that you can always verify the correctness of the content, because you
can compare the content that you get with the content you are asking for based on the
hash of the content.
And you are free to accept it from anyone and everywhere, which is great.
And it can be in a different network than the internet.
So it must not be the internet where we store the information.
It can also be in another network that is based on a completely different protocol.
So there are a lot of basic advantages of this.
But actually, the IPFS addresses these problems here compared to the World Word Web.
It's verifiable, so you can verify the information.
It's resilience because we store it at different places.
It avoids centralization of big, big platforms.
It has quite a good performance.
It avoids link rods because you point to a web page that doesn't exist anymore.
On the other hand, we will see later on that in the IPFS, it could also be the case that
information is no longer in existence, that you think it is in existence.
We have data sovereignty and ownership.
You store the data on your own local server.
We have off-chain storage for blockchain, which means that the IPFS is actually the
media that is being used to store information that we refer from the blockchain, smart contracts,
and the NFTs.
I mean, that's the big example for that.
So we have NFTs.
They do not store image information.
We store this kind of information in the IPFS.
And we have local first software, so store information locally first.
And we don't have a vendor looking because this is really an open protocol.
You don't need to rely on a particular vendor to use that.
And all in all, the whole IPFS has different components.
So we have components that represent and organize the data.
So we have the content addressing scheme.
We do this, for example, in decentralized, iCyclic graphs.
This can be in JSON or things like that.
This is just the component to store the data.
We have information to find data for the content routing, which is decadently DHT.
We discussed this briefly today.
And then we have things to transfer data.
This can be done with HTTP, but we can also use BitSweb or any other kind of protocol.
We need to do some kind of addressing of the data.
This is the standard with multi-forwards.
And then we have the bridge between IPFS and HTTP, which is an IPFS gateway.
And you may remember that during the last lecture, I was showing you some NFT images.
And I was just using my normal browser to download these images.
And this was actually done by an IPFS gateway.
So I was just using a browser.
There is an HTTP address, which contains a normal HTTP address, together with a hash.
And then this gateway retrieves the information from the IPFS and shows us in the web page.
And then we have here to peer connectivity, which is the libp2p.
And we have some kind of dynamic naming and things like that.
These are just subcomponents.
Today, we will basically discuss the first two things here.
How do we represent information?
So we discuss about this one and we discuss this one here.
OK, so again, to compare the two different concepts for resolving the data online, just to sum this up,
we have location-based addressing, which is the URL.
So we identify by the storage location, www.fitfrauenoverde.es, slash something.
That's coming.
Can I ask a question?
So is there a question?
Yes, can you hear me?
Yeah, I need to just make it a little bit louder.
Just a second.
OK, go ahead.
Yeah, I want to ask a small question.
So you said that the files can, in principle, be stored, for example, in just one server and that they must always exist.
But if, for example, that server is down and some file is lost, let's say, irreversibly, it was deleted,
like, will this be a problem for the system operation?
Yeah, exactly.
That's the case.
So the IPFS is not a storage system for infinity.
So if you store something on your local IPFS server and then you show, then you send and then you give me the CID,
which is the link then to this particular file and then you shut down your server.
And if nobody else, in the meantime, retrieved it and provided a copy on its own server, then it's gone.
So that's possible.
But there are other systems that are based on the IPFS where you somehow pay for the information that is being stored there.
And then they make sure that the information stays there.
And last time, for example, Filecoin goes in this direction and then there are other things.
We have a list at the end of the lecture where you see what other kind of systems exist,
where you can make sure that the information is persistent and cannot be just removed from the system by shutting down a server.
OK, thank you.
So that's really the case, that you have to remember.
Putting information on the IPFS guarantees that when you retrieve it, it is the information you are asking for,
but there is no guarantee that you get it.
OK, so yeah, we have this kind of address-based thing.
We have the authenticity is provided by trust in the location owner.
So because you really trust that when you retrieve a web page from the arbitrary Aachen, that it's true what's up there.
And we retrieve this with the DNS, the dynamic name service.
With the content-based addressing, we identify data only by the content, not by the location.
There is nothing like rwth.de slash lecture xyz.
So we do a self-certification of the content.
You self-certify the content via the hash in the address, not by the trust and the provider of the information.
And he will retrieve the whole information with peer-to-peer networks.
So let's have a look first at the kind of, whoops, the mouse.
OK, content identifiers.
So content identifiers, that's the question, how do we identify content within the IPFS?
So and there we have to answer four different questions.
The first question is, if we hash information to make it addressable, how do we feed this hash algorithm?
How do we serialize the data?
That's the first question.
The second question is, what kind of hash algorithm do we use?
Then we may have different schemas and how do we represent the result?
So these are the four questions we need to answer when we would like to store information in a system like the IPFS.
So the first question is that really, how do we feed the algorithm?
What we actually need is that we have a strict way and a standardized way to take a file in order to hash it.
And now you may wonder, OK, what's the problem up there?
So for example, if I have a JPEG, I have a JPEG and I can hash this JPEG.
That's true.
But what happens if you have, for example, more structured information like a JSON file?
And it may be that when you have a JSON file that different libraries that save a JSON file to the disk.
And if this JSON file represents, for example, a tree or an array,
it's not always guaranteed that two different libraries store this JSON file in the same sequence.
Imagine that you have a tree.
A tree, you can go always to the left and down, but you could also always go to the right and from the right to the left by sequencing a tree.
So with a linear file format, that's not a problem.
But if you have more complex data, or imagine that you, for example, you have a big 3D image.
There was a big 3D scene.
There are different objects in this scene, but how do you serialize that?
So we need to have a standard to do that.
Because obviously, if you serialize a tree from left to right,
you get a different hash than serializing it from right to left.
So we need a standard for that.
So and this is done by this multi-code extended.
And for example, what we have is this list.
And for example, we have this example codec 78.
This is a plugin to serialize the data structure of a Git object that you know from GitLab or GitHub,
where we store software sources.
And here it's also the same.
If you store a big software project, how do you store that?
Do you go from first the directories and you store the directories by the alphabet?
And then you store the files?
Or do you first serialize the files and then the content of the directories?
And these codecs you can see here.
And we can click on, oops, let's see if I go back to the pointer.
Then this page opens and we just show you this page here.
Just over here.
So you see and there you see on this GitHub page,
you see all the different codecs that exist and that describe how to serialize a particular file format.
So if you have a very special file format, you should look this up and then you see how to serialize that.
If you have a particular client for the IPFS, then this is done automatically.
But this is what we need to store.
So we take our data, we serialize it according to that and then we store it on the IPFS.
That's the first thing.
The next question we need to answer is what kind of hash algorithm do we use?
I mean, we can use SHA256, we can use SHA128, MD5 or whatever.
And also this must be specified.
I was always saying, well, you just hash the data and then you use that as the address.
But what kind of algorithm do you actually use?
And this is also then being stored in this content identifier.
So we have already two things that we store the content identifier.
The first is how did we serialize the data?
The second thing is what kind of hash algorithm do we actually use?
And the next thing is then how do we represent the result?
So, for example, we all know that the Germans have these strange characters, the umlaute.
Or you have other kind of very special characters.
And you all know that sometimes in some documents, in some programs, these kind of umlaute,
they just show up in a very strange way.
Or even they got lost because they have not been able to be transferred using the underlying network system.
And therefore, we need to find a way to encode the data such that it can be transported in a very easy way.
And that it can be understood by all computing systems in the world.
And normally what we do there is base 65.
This is the case.
That's a nice algorithm that we can use to take any kind of data that also contains a lot of binary data
and to represent it as very simple characters.
The big disadvantage is that we very often blow up the data because binary data is much more compressed.
And then when we transfer binary data into base 65, the data becomes very, very large.
That's the disadvantage.
But the advantage is it can be interpreted by all the different computing systems.
And it's easy to handle then in the end.
And again, you also see there the link.
And when you go to the link, then you also see the different kind of formats.
And if we now put all that together, then we have the content identifier.
So the content identifier then says, well, we have the base, how we encode it.
So how we do the base encoding, what kind of coding we use and what kind of hash we use.
And that allows us now to have these keys here, to have these kind of links.
And these links, they contain all the information in an encoded way.
And normally this is base 65.
So you see, you would say, well, where do I find here the hash algorithm?
And where do I find the codec?
Well, you can just click on that.
Then again, we get a web page.
And I can show you the web page here.
And then you can see that in this web page, we see this is base 58, BTC encoded.
It's a duck.
So it's a Merkel tree that we actually have encoded here.
And the actual hash of the whole thing is this.
So, and we also see that it's being hashed with the char 256.
So this is what's being encoded in this particular string here.
And you see, this is the, here is that this was version number zero.
And then when you always see this QMB, you can see, ah, this is probably encoded by version
number zero.
And in the meantime, we have version number one.
And both represent actually the same link.
But this is encoded by version number one.
And when we click on this, and we get the web page coming up, which I can show you here.
And you see, that's the encoded link.
And it's again, this is now base 32.
Here is the hash.
And when we look at the hash, then we see it's the same.
So you see both links, they actually address the same content in the IPFS, but in a different form.
So, but what we got with this is actually now that we have the content identifier.
And this is how it looks like.
And that's how it's being constructed.
Normally, you don't, if you don't start programming on the IPFS, or if you start
programming to have your own content identifiers, you don't really need to deal with that.
This is all done by the system itself.
But this is to understand how this is being constructed.
And you could also use this kind of algorithm if you would like to address any other kind of
information that you probably do not store in the IPFS, but that you would like to point
your friends to that you store on your local file system or whatever.
Okay, so that's the first thing, the content identifier.
The next thing is now, how do we actually store the data on the IPFS?
And that's also quite interesting.
So what we need to do is that we need to store the data in the IPFS.
And we discussed already that you need to start different kinds of data.
So we need to store JPEG images, video images, MP4.
But we also need to start, for example, the whole directory where we store our sources.
So imagine your Rodeonize program where you have different sub files and different kinds
of HTML pages included.
And we also need to store that.
And for that, we need to find a way how to represent this actually in the IPFS.
So we have already this way how to serialize that.
But the next thing is that we say, okay, when we have a very, very large data file,
let's say a big, big image, your sources, or any kind of movie or whatever,
we say that we chunk it into blocks, which means we are not storing a big data file
as a single block in the IPFS.
So the protocol says large files need to be chunked into blocks.
We see later on why this makes sense.
What we then need to do is that we say, okay, we have, for example, one megabyte of image
and we chunk it into 100k blocks, then we need to generate a CID for each block.
So that's what we need to do.
So each block then has its own content identifier.
And the files that we have been chunking, then in addition being organized in a tree.
So this one megabyte file that we chunk, for example, into 10 blocks is then a tree with 10 leaves.
That's the way how we do that.
And the properties of this Merkle duck are then that the CID of the parent node depends
on the CID of the child node, similar to the Merkle tree that we discussed when we organized
the transactions for the Bitcoin blocks.
And if we change then leave content, all nodes from the leaf up to the root change also their CID.
And we can also say that if we have the content identifier of two Merkle trees,
if both are equal, then the trees are equal.
That's the case because the parent CID depends on the leaf CID.
And therefore, if we have the root CID of two Merkle trees, if they are equal,
then the content is also equal for both.
And also with the root CID, we can retrieve the whole tree.
So if we have the root CID, we can retrieve the whole file, which is necessary,
because when we chunk it, then we need to make sure that in the end,
when we just have the top node, we need to be able to get it.
Okay, so that's the way how this works.
And what you see here is then that there are some kind of explorers also.
And you see this here in this case, that we have a particular file.
And this has been chunked into two different sub nodes.
So we have the file, that's the root node for the file.
And here are the two sub nodes for this file.
So actually this has been chunked into these two different sub files.
So that's how the way how we store information in the interplanetary file system.
So we don't put a one megabyte file as a single chunk onto the file system.
We chunk it into sub blocks, which are then organized in a tree.
So the question is now, and maybe you were wondering, why do we do this?
Do you have any kind of information why we do that?
Probably on the one hand, it must be much easier if we just store it in a single piece.
Why do we do all this hassle of chunking it and building trees out of it and things like that?
Do we have any idea why we do that?
Yeah. Go ahead. You just need to unmute yourself then.
Well, I'm not sure, but given the decentralized way of how we store things,
it may be easier to store one chunk at one server and another chunk at another server
and don't block an entire server by, I don't know, storing a terabyte project.
Yeah. Yeah. That's a good idea.
So we have a lot of parallelization then, which improves it.
So that's one good argument. You could say that when you retrieve a one megabyte or one
terabyte file, that you actually can retrieve it not just from a single source,
but you can open up 10 connections to 10 different servers and then you have a parallel retrieval
of the content. That's a good argument. There's another argument for it. So we have one.
There is another argument. Someone has another idea.
But maybe I misunderstood something, but as I understand, the files can go from one server
to another if another wants to retrieve those files from the system. But why would I, for example,
want to retrieve a few blocks and not the whole file? You referred to the explanation that I just gave.
Yeah. Well, the reason is the following. If you retrieve this from a, if you retrieve it just
from one server, you know, you completely depend on the bandwidth to this particular server.
And in the end, you just do this then sequentially more or less. But if you can retrieve
10 parts or 10 different parts of the same file from 10 different locations, you can parallelize
that. And then you may get it in one tenths of the time. You wouldn't get it in one tenths of the
time. But maybe you get it, for example, in half of the time, then just retrieving the whole block
from one system, from one single server. But I have a question on this. But if like, okay,
one server is down, then that, like if you piece it in different servers and it's down and then
you lose that piece, so to say, right? And then you can't retrieve it at all.
That's also the case. Yeah, yeah, definitely. If you have split up your file into 10 different
chunks, and these 10 different chunks are located on 10 different servers, and one of them is down,
and you don't have any replication of that, then you wouldn't get your whole file. That's true.
Yeah. But we are thinking here about somehow about an ideal world where all the different
servers are up and running, which is not always the case. But it could happen. It could happen.
What I would like to show you to answer the question is this, I just point here.
Well, I just retrieved that from the image. And this gives an answer why
the, it gives another question. So the first question was that we somehow can distribute it
over different files, so I was that we may get a faster reply of that. The other reason why we
chunk the information into different paths is that we may reuse the same information.
Because if we have a file that contains information that has already been stored
in our distributed file system several times, let's say, a very often being used meme, an image,
a standard text or whatever, then we can refer to this particular chunk.
So imagine that you have, you are doing a lot of software development and with software development,
you always include the same library within your sources. And this would mean that you have the
library already somewhere in the IPFS, and you only then need to link to this particular library.
You don't need to store this within your large content as a single blob.
And that saves this storage. And by having then multiple references to that, which would also
mean that all the software system files are there, maybe even retrieved on different servers,
also increases the redundancy of that. But there's a question.
And is this done automatically or do one have to manage that?
No, this is done automatically. I will later on show you the IPFS desktop client,
and then we see how this is done completely automatically.
Okay, so we have now two things. We know how to address files in the IPFS.
And we also know how to store them in this Interplanetary file system.
And now we get to content routing. And content routing is now quite a complicated thing.
And I mentioned that already at the beginning, there is a link to a YouTube video of someone
who explains it very nicely, which I can only recommend. And I just recommend also to listen
to that. And I will try to give you an introduction on how that works.
Normally, I use the whiteboard for that, which is unfortunately not possible now.
But I hope I can get you somehow an idea on how this particular thing works.
The actually challenge that we now have with our IPFS is after we know how we do the addressing
with the content identifier, and after we know that we chunk files into different pieces,
where do we store them? And how do we find them in the end?
So think about the World Wide Web. When I give you a web address,
then based on the DNS, this name is being resolved into an IP address. And based on the IP address
and the DNS, people know where the server is located, and then they're just forward your
request to this particular server. That's done in the Internet with the HTTP protocol works fine.
With the IPFS, we have the challenge that so far, the addressing of content is just more or less
the hash. It's the CID. There is no information where to find it. How do we find the server that
is actually storing that information? There is no server address within our content identifier.
How can we find the server? And that's the challenge. And that is now being solved with the so-called
cadamilla distributed hash table algorithm, which we discussed there are other kinds of gossip based
routing, which we will not discuss here. This is just for naming it. But that's the solution now,
how we find the information within the IPFS. So the basic architecture is the following. We have
a lot of different servers running peers in our context here. And each peer has a unique ID,
which is this public key. It's somehow his public key. See just as a name or see it as his address
within the IPFS. And this peer is discoverable. So you can ask the peer, are you alive? Are you
not alive? And there are some peers which act somehow as some kind of first addresses where you go.
So some kind of anchors. And they know all the other peers that are around. We see this when we
look at the desktop application. And it must be reachable. So and it can also be reachable by
different protocols because we have this multi address protocols. So if it's not reachable,
then we can't get the information that is being stored on the peer. You mentioned that already.
We have some kind of encrypted communication. So the information that we transfer between the
different peers is being encrypted. And the important thing is that the content is not
replicated automatically. And here we have the big, big difference between the peer to peer
systems that we know from years ago, where we used peer to peer file systems to share music files.
So when I was younger at your age, yeah, so there was not something like Spotify or whatever.
What we did is, you see someone joining, what we did was actually that you could
share music with a peer to peer file system, which was illegal, because there was copyrighted music.
And the big danger was the following, that when you participate in this network,
just by participating in the network, your note that you are running on your PC was automatically
providing information to other nodes without your knowledge. And in particular,
this note was also retrieving information from other nodes without your knowledge.
So after one hour of having a server up, it was already heavily providing copyrighted
information to other nodes. And if this was detected, you assumed and you had to pay a fee.
So this happened for several years. People use several tricks on that to avoid that.
But this is not the case in the IPFS. In the IPFS, you don't need to be afraid
that your note is suddenly providing information to the outside world that you have never retrieved.
So there is no danger that you have suddenly you're providing copyrighted information,
pornographic information or whatever can happen, unless you downloaded it yourself.
So that's the case. As soon as you access a file on the IPFS, then it's also being stored on your note.
So then you also become a provider unless you delete it from your note.
So but that's an important thing, which makes it also very important for using that
system for companies. Now, for example, for Fraunhofer, I would never be allowed
to run such a note if there is that danger that suddenly we provide any kind of illegal
information by this system, such as that we become a content provider for illegal information.
Now, the next thing is now that the point is that
how do we store information about what is being stored in the IPFS? I mentioned already.
Imagine that we need that I upload a file to our system to the IPFS. How do other people know
about the information that I have uploaded there and how do they find it?
So the scenario is the following. I upload a file to the IPFS. It's being stored just on my server.
And then I provide you the CID to retrieve the file.
How does your local server know to find my server where the location, where the information is
actually being stored? And this happens by the following. As soon as I upload a file to the
local server, my server informs other notes that it has contact to about this new information.
So what it does, it actually tells that my neighbor notes that I have uploaded information.
But it's not telling everybody about that. It's telling only those notes about this information
that are closest to the information itself. So this is a little bit mind boggling. What does
closeness mean here? Closeness does not mean that it stores this information at the notes
which have the shortest geographical distance to me. So it's not storing the information.
For example, imagine that my neighbor just across the office here has also an IPFS running.
It does not mean that it's informing this note about the fact that I have uploaded,
let's say these PowerPoint slides to the IPFS. It informs the notes that are closest
to the content identifier of these PowerPoint slides. And this is being computed, the closeness
is being computed by performing the XOR operation. So what we do is actually that we
do an XOR operation of the content identifier of the data and the address of the note.
And those where the XOR operation of these two addresses, the content identifier and the
identifier of the note becomes the shortest. They get the information that they can find.
This information on my note, which is an address by the IP address. So what we actually do is,
and you see this here and there you see an image of the Cadamlia explained YouTube video and you
will find this also then in the slides. He does a very nice job in explaining that from the very
beginning on. So the video is about half an hour long and I really recommend to look this up also.
What you see here is in a very, very simple example,
we just built the namespace of four bits to simplify it. And what you see here is that we
saw somehow information. So this is information, the blue dots are information. The pointer here,
so the blue dots are information and the other dots here, this is a computer.
Yeah. And what we do actually is that we place both the information as well as the notes into this
namespace. So for example, if I upload the PowerPoint slides here, then the PowerPoint slides,
they have a content identifier. In this case, it's just a four bit content identifier 0 1 0 0.
And the information about that we store at the closest nodes, which have to the closest nodes.
And the closest nodes is N five, because the difference between 0 1 0 0 and 0 1 0 1 is 1.
So we store the information about our PowerPoint slides, we store them here at that particular
node. Plus some other eight other nodes. So we would also store it here and there, for example.
Yeah, just to have some kind of redundancy. You see another example that the distance between
that node and that node is one. So it's eight. So it's 1 0 0 0 and for digits, so it's eight in
and in decimal numbers. So the actual way how this works is that we upload the information.
And then we inform the nearest nodes based on the address that we have uploaded this information.
So if you now want to find this information, your notes. So I provide you now the content identifier.
And the content, your server takes the content identifier and looks up his own table,
which is the closest node to this content identifier. So he builds the XOR operation
of the content identifier and the nodes that the client knows about. Then he's asking the closest
one, do you know more about this particular data? And if this node does not know more about it,
again, it will ask the nodes in its own system, in its own cache, who is the closest one to this
content identifier. And this in general takes two to three operations. And then normally,
a node is found that knows about my server. So it knows the IP address of my server.
And then it returns the IP address of my server to your node. And then you can do this by appear
to peer protocol to retrieve the particular information. Question. Yeah. Go ahead. So when
I'm uploading information to my server, I'm telling the nodes that I already know,
and that are the closest to that information hash that I have this, because otherwise, I would
need to know every node address, right? Exactly. When you start a server, then you only see
some nodes. And I think the easiest way is now that we have a look at this desktop, because
so much theoretical information now, we just play around a little bit with that.
So what you see here is, and move it over here to this particular desk.
But now you should see my IPFS desktop. Actually, you can download an IPFS desktop,
just look for IPFS desktop. Let me see if I still have the web page open, because I had to,
because I had to retrieve the latest version recently.
I am here. No, that's the web page where I downloaded it. So you see github.com,
IPFS, IPFS desktop releases. And the count release here is the version 0.47. And there you can
download the different things. And the easiest way to download it is really here to use here,
this Windows file. Now download it, double click on it, and then you can install it.
So I just want to do is just copy it. So if you like, there is my go, I put it in the chat.
Here's the link. And then, for example, if you like, you can just retrieve it,
download it on your laptop, and then you are part of the IPFS network.
If it's running. Okay, so, and this is now the desktop. So we can see what is this current status.
So there are currently 28 megabits on data that is being hosted on my server.
We detected 10 other nodes in the IPFS. That's my PID. So you see, it looks very much like
content identifier. And my agent is Kubo. That's the particular implementation.
And here you see my current bandwidth, what I did. And you see currently nothing is going on here.
So at the moment, I'm not providing any kind of information to anybody. And also I'm not
retrieving any information. So let's see, we can look what do we see here. There we see all the
addresses of the different nodes that we are being connected to. So then I can have a look at the
different files. And here are the files that I have currently uploaded. So here are the files that
I uploaded. So that's a 3D printer file that I uploaded just as a test. And what I can do is that
I can also just remove it. Okay. So this means that I'm no longer providing that information to
the IPFS. And if I would have provided you a link to that one, it would probably no longer be
available for you unless someone else downloaded it and put it in his own IPFS server. So we can
look at the files here, click on it. And then you see that's a introduction to a book that we
just published recently about the Web 3. And I uploaded it here. Actually, it's a preprint
of the book. And I put it here on the IPFS. So that's it. And look at it. And here we have another
chapter that I uploaded there. Okay. So that's the way how I put it on the IPFS. And you see here
that's the, what I do is that I can take the link. So I copy it. And I put the link here into the,
into our chat. So you may now use this particular link to click on it. And maybe even your browser
may be able to open it. But it may also be possible if you like, you can just take the CID
like that. And you can paste this CID. If you have a site job downloaded the IPFS desktop,
just put it here into this particular field. And then you can retrieve it.
And then your server or your client, your desktop would retrieve it from my client.
Maybe someone tries it out. And then we see if this happens. Let's see here at the status.
Yeah. I am there is something, some traffic coming in. Yeah. Is somebody already trying?
Do that? Yeah, I just downloaded it. You downloaded it. Okay, perfect. So this seemed to work.
So what else can we do? We can have a look at the other files. So it says here you can explore the
Merkle tree, the Merkle forest. But we can also go back to this one here. And if we take here,
and I think you could do the same now in parallel, when you have downloaded this,
you can say, let's investigate it. And what you see now is that you see the image that we saw
already on the slides, that the system automatically separated my file into two sub files. So it was
chunked into two different blocks. And each block has its own content identifier. And we have the
content identifier of the anchor, of the root. So if we have different files, that's something
larger. Let's see. Oh, you see, and this has already been chunked into six different files.
If you like, you can also upload in any kind of information now in your system. And then
you can share the link here in our chat, and then other people can share it. And that's the way of
sharing. So it's a very nice way of sharing. On the other hand, it's really you don't need
a third party for doing it. You're not uploading it to Dropbox or somewhere else. You just provided
locally. And yeah, that's the way how it works. We can also have a look here at the peers. And then
you see that I'm currently being connected to 16 different peers. So for example, in the United
Kingdom or wherever. And if I would like to retrieve now some kind of information, then my system
would take the CID, send it to the nearest one based on the XOR operation,
which maybe somewhere in Mexico has nothing to do with the geographic topology. And ask this one,
do you know which kind of server is currently hosting that? And maybe it's then even going back
to your local client to retrieve it. Yeah, but it's really the case that the server in Mexico
is actually the storing this information because it's closest to the information.
Yeah, that's how this IPFS client works here. You see currently we have 709 peers, 751 peers
that are currently all over the world. And there are some companies that really operate larger servers
to provide some kind of redundant information in this particular net.
What else can I do? I can also do this kind of pinning.
Yeah, unheftened. So this means pinning. What I can do now is that I would like to pin this
information, which means if I just do it now, this kind of client that is currently running here on
my desktop, it's using, I can tell it, please use 10 gigabytes of my hard disk. But if I now
browse a lot in the IPFS, it will download a lot of information. And after some time, the 10 gigabytes
are full, then it will start to do some kind of garbage collector. And then it will remove files
from my server, which are probably stored somewhere else, but it will not remove those that have
pinned. So with pinning, I somehow tell the system, these are important files, and please do not
remove them when you do any kind of garbage collector.
Okay, that's someone. There's a question. Okay, go ahead.
Yeah, basically, I have multiple questions. So firstly, when we just launched the system,
I imagine that we don't know any sort of peers. So is there is like some separate mechanism so we
can identify our peers or some other servers? Yes, yes. So I would lie, I would say I really
know how it works. But I think what they do is some kind of broadcast. They do a broadcast
into the internet to find other kind of peers. And then when they find a peer, then this peer
is telling them, look, and also have this other kind of peers. So it's some kind of bootstrapping
that's taking place there. Okay. Then the question, like if the address of our server
somehow depends on the content, which we store which we provide, so should our address then
change when we upload new content or remove new content? No, no, the address of your server
is just being determined when you when you upload your server. Like, so it's actually a public key
public and private key that's being generated, which is just generated arbitrarily. And it's
absolutely independent of the content that you store. But I mean, how does the system work then
if we compare like using the XOR operation? Yeah. And how does it work then? It does the
following. Let's say if we just take the example that your server has been started and it became the
address number 4711. Take it very simple. So your server has now the number 4711. And I'm
uploading information that has the content identifier 4712. Then this is very close.
And then it will store the information that I have uploaded that I provide a file with the
content identifier 4712 at your server, because your server is very close to it. But this is just
by accident, just very completely arbitrary. You by accident got the address 4711 and I uploaded
content with the hash 4712. And it's just completely random that you then store this information.
But it's not just that you, it will also store the information at 4715 and let's say a 4600 or so.
That is really the point. When you first look at this cadamilla algorithm,
you really have to separate between where is the data being stored. And that has nothing to do with
the nearest and closeness. You store the data at the servers that have uploaded it more or less.
The information about where it's being stored, that is being stored at the closest
content closest server. But they don't store the information itself. My file 4712 is not
being stored at your server, just the information that you can find it at my server is being stored
at your server. Okay, so good. So we are back on the slides.
Okay, so this is also being very nicely stored here in this video that I can recommend.
And then, yeah, I think what I just explained is also explained here in this particular system
and in this particular slides. So the provision example, I provide data to the IPFS. So I compute
the CID of the data. Coming. Okay. So I compute the data. Okay, so I just compute the CID of the
data 4712. Then I look up in my connection table that it nearest to my data. This would be 4711.
So it would be your server. And then I ask your node to store the lookup information
for my information. And if you have now peers that are closer, let's say you are 4711,
my data is 4800. And then at the moment, you are the closest to me. So I send it to you 4711.
And then in your lookup table, you have a peer that has the number 4801. And then you say,
okay, I know a much closer one. I send it to this one.
And that's the way how it works. So uploading means telling all the closest nodes about where to
find the information. And when you look up information, then I asked my neighbors for the content.
So I would ask you now, if I look up something that has the address 4700.
Yeah, I know you as 4711. I would like to ask you, then do you know this? And if you don't know that
you look up the nearest node to this one. And then this node provides me the information.
That's the IP address to use BitSwap to retrieve the actual content.
So in summary, actually, the content routing is based on this cadamilla DHT,
which existed already beforehand. So cadamilla was already available before IPFS has been invented.
And using the BitSwap protocol to retrieve the data. So it's a peer-to-peer protocol.
The data is stored distributed at PIDs that are close to the key in the hash table row.
And every node maintains this information. And in general, about 20 buckets of this address
prefixes. And with this 20 buckets, the experience has shown that with up to two or three hops,
we actually find the provider of the data. And what you do is you really ask the closest ones.
You ask the nodes that are closest to your data. And then you can use the data to retrieve the
to your data. And then actually, you will find it.
Yeah. To sum up, what you should remember actually is the whole thing about the CID,
is this content-based addressing. These are immutable links,
which allow unique identification of self-certification of data. So it's a sentence with
a lot of interesting things. Immutable links. So when you modify the link, you wouldn't get the
data anymore. Link and data are married to each other. You have unique identification and you
have a self-certification of the data. Because when you retrieve the data and it doesn't belong
actually then to the hash, then you can say, hey, somebody was cheating me.
And the file system, the interplanetary linked nodes, defines the standard to represent the
data structure as merkle trees. And the merkle trees is what you saw there in the desktop,
where you saw that the file that I've uploaded is really being split into 10 different chunks.
The distribution is, it's really a community system. As soon as you start running your server,
you become part of the community and providing this kind of information. And it's also being
that it's, as we discussed at the beginning, so you can, as more servers we have, the more
connections you have really to retrieve the file, which makes it faster, the whole system.
And we have some kind of deduplication benefits. So the same data results in the same CID.
You see, I've uploaded here the introduction of this book that we published, if someone else starts
publishing this book, the same file, it will maybe start on a server, but it's also being on my server.
And we somehow don't deduplicate the whole thing. Even if he then removes his server from the net,
it's still on my server. So on the same hand, we have redundancy like this.
The issues are definitely permanent storage. It may not be accessible every time and permanently.
Some of you mentioned that already at the beginning. What happens is the server is down,
server down is down. You don't get the information anymore. For that, you can use some kind of pinning
to avoid at least that it's being removed by the garbage collections.
Or, but there are two developments, Filecoin. It's the RBNB for storage. You can run a Filecoin
server on your own server. And by that, you somehow rent disk space to other people.
But you have to do it in a reliable way. And if you do that, you get some kind of crypto.
You get the Filecoin. So the Filecoin is the currency to pay you for providing storage in
the Filecoin network. And if you don't do it in the end, or if your server is not reliable,
you wouldn't earn any money. So it's very nicely to compare it with the RBNB for storage.
And our Weave is more a commercial operated system where you pay for uploading information.
And they guarantee that the information remains persistent on the server.
Yeah, there are some links which I can reprimand, which I can
recommend. There is the tutorials here on the Protoschool here on the IPFS.
And there are some other kind of nice interesting links to deepen the whole thing.
Okay, yeah, with this, I'm through the topic. We still have a little bit of time for questions
or clarifications or whatever. Do you have any questions?
Yeah, all right.
So in the example where you said that someone has the address 4711, your content has the address
4800. And then you say the 4711 client that here this is, you are the closest,
this is the data I have. And then they say, okay, I know someone at
4800 and one. So I tell them that there is the data. Do the 4711 people still keep your address
for that information? Yes, they do that as long as I mentioned these K20 buckets.
So they store a certain amount of lookup information. And as soon as they are still,
if the information is still among the range of the closest information, they keep it.
Otherwise, they apply a garbage collection and they remove it because then they say,
I'm too far away from this information. I have already information about much closer notes.
Yeah, and then I remove this information because this is no longer really relevant for me.
Is that clear? And the size of the buckets is then if you have 20 buckets, it's 120th of
to the power of 256 or? No, it's just 20, 20, 20. I think it's 20.
But yeah, probably I would need to look this up. But they normally store 20,
20 other notes about this.
But you got me there.
Yeah. Because from how I understood it, if you have 20 buckets of the whole address space,
let's call it that way, it should the size of each bucket is 120th of the entire address space.
No. Yeah. And I think I would need to look this up. Let's discuss this next time. I check this
about this bucket size. Yeah, I think this is also being addressed in the video. He talks
about the bucket size also in the video. Any other questions?
Okay, if this is not the case, I will upload the slides.
Feel free to play around with the IPFS desktop. If you like it to share information among your
friends or to publish any kind of other things that you would like to share.
As I said, we have an institute meeting next Wednesday. Therefore, I wouldn't be able to come
to Aachen. Would it be okay if we have the lecture then again as somehow a Q&A session
next Friday morning again?
Yeah, for me no problem.
Okay, good. Okay, then I would, it depends if this strategy meeting is being cancelled,
which I don't think, then I tell you, but I think at least on Tuesday, I can inform you
about the situation. And then I will just publish the link to a video link. And that would then be
the last lecture, I think, and then we can just have a Q&A session. And I probably can just go
through the slides and you can answer your questions. Or you can ask your questions. I
hopefully can answer them. And I will point out to some things that probably show up in the exam.
Okay, then thank you. Okay, then I stop the recording. That's important. I need to stop the
recording. Okay, good. So then
