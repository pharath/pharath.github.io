---
title: "Databases Notes"
read_time: false
excerpt: "Databases Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - databases
  - notes
---

# List of DBMS

[Ranking according to the popularity](https://db-engines.com/en/ranking)

# Types of Databases

- NoSQL Databases
  - Key-value database
    - Key–value cache 
    - Key–value store
    - Key–value store (eventually consistent) 
    - Key–value store (ordered)
  - Tuple store
  - Triplestore
  - Object database 
  - Document store 
  - Wide Column Store
  - Native multi-model database
  - Graph database
  - Multivalue database

# 7 Database Paradigms

**source**: [Fireship](https://fireship.io/lessons/top-seven-database-paradigms/)

| DB Type | Popular |
| :--- | :--- |
Key-Value | Redis, Memcached, Etcd
Wide Column | Cassandra, Apache HBase
Document Oriented | MongoDB, Firestore, CouchDB
Relational | MySQL, Postgres, SQL Server, CockroachDB
Graph | Neo4j, DGraph, Janus Graph
Search Engine | ElasticSearch, Algolia, MeiliSearch
Multi-Model | FaunaDB, CosmosDB

# Structured Data

- data incorporating **relations** among entities and variables, [Wikipedia](https://en.wikipedia.org/wiki/SQL)

# SQL

Wikipedia:

- Originally based upon **relational algebra** and tuple **relational calculus**
- SQL consists of many **types of statements**, which may be informally classed as **sublanguages**, commonly:
  - a data query language (DQL),
  - a data definition language (DDL),
  - a data control language (DCL),
  - and a data manipulation language (DML).
- The **scope** of SQL includes
  - data query,
  - data manipulation (insert, update, and delete),
  - data definition (schema creation and modification), and
  - data access control.
- Although SQL is essentially a **declarative language** (4th generation programming language), it also includes **procedural elements**.

# PostgreSQL

- CLI
- see [ubuntu.com](https://ubuntu.com/server/docs/databases-postgresql)

```bash
# start the server
sudo apt install postgresql-12
sudo pg_ctlcluster 12 main start
sudo pg_ctlcluster 12 main stop
sudo pg_ctlcluster 12 main status
```

```bash
# in another terminal

# connect to a database (opens a SQL Shell)
sudo -u postgres psql template1
```

```bash
# in the SQL Shell:

# list all databases
\l

# switch the current database
\c database-name
# or
\connect database-name

# to show all tables in the current database:
\dt
# If you want to display also the size and description of the tables, use the following command:
\dt+ 
```

```bash
# in pgAdmin or SQL Shell:

# run SQL commands (lowercase/uppercase does not matter)

CREATE DATABASE database-name;
DROP DATABASE database-name;
SHOW DATABASES;

CREATE TABLE College(cName text, state text, enrollment int);
DROP TABLE College;
INSERT INTO College VALUES ('Stanford', 'CA', 15000);

SELECT * FROM Student;
# but maybe:
SELECT * FROM "Student";
# press <tab><tab> to find out
```

## pgAdmin

### pgAdmin: Install and Setup

From: [ubuntu.com](https://ubuntu.com/server/docs/databases-postgresql)

To connect `pgAdmin` to a database `template1`:

```bash
# connect to a database (opens a SQL Shell)
sudo -u postgres psql template1

# in the SQL Shell:
# configure the password for the user postgres
ALTER USER postgres with encrypted password 'your_password';
```

After configuring the password, edit the file `/etc/postgresql/*/main/pg_hba.conf` to use `scram-sha-256` authentication with the `postgres` user, allowed for the `template1` database, from any system in the local network (which in the example is `192.168.122.1/24`):

```bash
# insert this at the bottom of file /etc/postgresql/*/main/pg_hba.conf
hostssl template1       postgres        192.168.122.1/24        scram-sha-256
```

Then open `pgAdmin`, click on "Quick Links" &rarr; "Add New Server" and enter the following configuration options:
- Host name/Address: `localhost`
- Port: `5432`
- Maintenance database: `template1`
- Username: `postgres`
- Password: `your_password` (the one which was set above)

In `pgAdmin` right-click on the server name &rarr; "Create" &rarr; "Database ..." to create a database.

Related:
- [tecmint](https://www.tecmint.com/install-postgresql-and-pgadmin-in-ubuntu/)
- [psql: create a database](https://www.tutorialsteacher.com/postgresql/create-database)

### pgAdmin: Query Tool

- [Query Tool](https://www.pgadmin.org/docs/pgadmin4/latest/query_tool.html)
- [Shortcuts](https://www.pgadmin.org/docs/pgadmin4/development/keyboard_shortcuts.html#query-tool)

# Query Processor

- **query processor:** the group of components of a DBMS that turns user queries and data-modification commands into a sequence of database operations and executes those operations
- **query execution**: the algorithms that manipulate the data of the database

# Scanning Tables

- **"scan" a table**: bring into main memory each tuple of some relation
- 2 basic approaches to locating the tuples of R:
  - **table-scan**: get the blocks containing the tuples of R one by one
  - **index-scan**

# Indexes

- definition:
  - any **data structure** that takes the value of one or more fields and finds the records with that value "quickly."
  - In particular, an index lets us find a record **without having to look at more than a small fraction** of all possible records.
  - The field(s) on whose values the index is based is called the **search key**, or just **"key"** if the index is understood.

## Dense Index

- **dense index**
  - a **sequence of blocks** holding only
    - the **keys** of the records and
    - **pointers** to the records themselves
  - The index blocks of the dense index maintain these keys in the same **sorted order** as in the file itself
  - keys and pointers presumably take much **less space** than complete records
    - many fewer blocks for the index than for the file itself
      - thus, faster to search through the index than through the data file
  - especially advantageous when the dense index file, but not the data file, can **fit in main memory**
- method
  - Given **key value K**
  - search the index blocks for **K**
  - follow the associated pointer to the **record with key K**

## Sparse Index

- **sparse index**
  - has **only one key-pointer pair per block** of the data file
  - thus uses **less space** than a dense index, at the expense of somewhat **more time to find a record given its key**
  - can only use a sparse index if the data file is sorted by the search key
- method
  - find the **record with search-key value K**
  - search the sparse index for the **largest key less than or equal to K**
    - Since the index file is sorted by key, a **binary search** can locate this entry
  - follow the associated **pointer** to a data block
  - **search this block** for the **record with key K**
    - the block must have enough format information that the records and their contents can be identified

## Multiple Levels of Index

- "putting an index on the index"
- first-level index can be sparse or dense
- second and higher levels **MUST** be sparse
  - **reason**: a dense index on an index would have exactly as many key-pointer pairs as the first-level index, and therefore would take exactly as much space as the first-level index
- idea has its **limits**
  - prefer the B-tree structure over building many levels of index

# NoSQL Databases

Wikipedia:

- A NoSQL (originally referring to "**non-SQL**" or "**non-relational**") database provides a mechanism for storage and retrieval of data that is modeled in means other than the tabular relations used in relational databases. 
- Such databases have existed since the late 1960s, but the name "NoSQL" was only coined in the early 21st century, triggered by the needs of Web 2.0 companies. 
- NoSQL databases are **increasingly used** in big data and real-time web applications. 
- NoSQL systems are also sometimes called "**Not only SQL**" to emphasize that 
  - they **may support SQL-like query languages** or 
  - **sit alongside SQL databases** in polyglot-persistent architectures.
    - **Polyglot persistence** is a term that refers to using multiple data storage technologies within a single system, in order to meet varying data storage needs.
- **Motivations** for this approach include 
  - simplicity of design, 
  - simpler "horizontal" scaling to clusters of machines (which is **a problem for relational databases**), 
  - finer control over availability and 
  - limiting the object-relational impedance mismatch. 
- The **data structures** used by NoSQL databases are **different** from those used by default in relational databases, making **some operations faster in NoSQL**. Examples of NoSQL Data Structures:
  - key–value pair
  - wide column
  - graph
  - document
- The particular **suitability** of a given NoSQL database **depends on the problem** it must solve.
- Sometimes the **data structures** used by NoSQL databases are also viewed as **"more flexible" than relational database tables**.

# Graph Data

## Graph Databases

[Wikipedia](https://en.wikipedia.org/wiki/NoSQL#Graph):

- **Graph databases** are designed for data whose relations are well represented as a **graph** consisting of elements connected by a finite number of relations. 
- Examples of data include 
  - social relations, 
  - public transport links, 
  - road maps, 
  - network topologies, etc.

Graph Databases (by popularity) (see full [ranking](https://db-engines.com/en/ranking/graph+dbms)):
- Neo4j 
- Microsoft Azure Cosmos DB 
- Virtuoso 
- ArangoDB
- OrientDB
- Amazon Neptune	
- JanusGraph
- GraphDB
- NebulaGraph
- Memgraph

Graph query-programming languages [Wikipedia](https://en.wikipedia.org/wiki/Graph_database#Graph_query-programming_languages)
- AQL (ArangoDB Query Language)
- Cypher
- GQL
- GraphQL
- Gremlin
- SPARQL

# Semi-Structured Data

- xml
- json

# xml

## DOM Parser vs SAX Parser

From [stackoverflow](https://stackoverflow.com/a/6828897/12282296):

In just a few words...

SAX (**S**imple **A**PI for **X**ML): Is a stream-based processor. You only have a tiny part in memory at any time and you "sniff" the XML stream by implementing callback code for events like `tagStarted()` etc. It uses almost no memory, but you can't do "DOM" stuff, like use xpath or traverse trees.

DOM (**D**ocument **O**bject **M**odel): You load the whole thing into memory - it's a massive memory hog. You can blow memory with even medium sized documents. But you can use xpath and traverse the tree etc.

## XPath

## XQuery

## XSLT

# json
