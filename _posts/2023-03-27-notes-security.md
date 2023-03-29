---
title: "Security Notes"
read_time: false
excerpt: "Security Basics"
toc: true
toc_sticky: true
categories:
  - Notes
tags:
  - security
  - notes
---

# Authentication

## Amazon Cognito

see [What is Amazon Cognito](https://docs.aws.amazon.com/cognito/latest/developerguide/what-is-amazon-cognito.html)

see [Pricing](https://aws.amazon.com/cognito/pricing/)

**MAU**: Monthly Active Users
- A user is counted as a MAU if, within a calendar month, there is an identity operation related to that user, such as sign-up, sign-in, token refresh, password change, or a user account attribute is updated. You are not charged for subsequent sessions or for inactive users within that calendar month.
- There is separate pricing for users who sign in directly with their credentials from a User Pool and for users who sign in through an enterprise directory through SAML federation.
    - SAML
        - [Wikipedia](https://en.wikipedia.org/wiki/Security_Assertion_Markup_Language)
    - OIDC
        - [Wikipedia (de)](https://de.wikipedia.org/wiki/OpenID_Connect)
        - [Wikipedia (en)](https://en.wikipedia.org/wiki/OpenID#OpenID_Connect_(OIDC))
            - It allows computing clients to verify the identity of an end user based on the authentication performed by an authorization server, as well as to obtain the basic profile information about the end user in an interoperable and REST-like manner.
