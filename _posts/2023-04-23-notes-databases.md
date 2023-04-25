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

NoSQL
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

# NoSQL

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
- The **data structures** used by NoSQL databases (e.g. key–value pair, wide column, graph, or document) are **different** from those used by default in relational databases, making **some operations faster in NoSQL**.
- The particular suitability of a given NoSQL database **depends on the problem** it must solve. 
  - Sometimes the data structures used by NoSQL databases are also viewed as "more flexible" than relational database tables.

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
