# AI Coding Instructions: [Project Name]

## 🎯 High-Level Context
- **Domain:** Brief description of the business domain (e.g., Fintech, E-commerce).
- **Architecture:** (e.g., Clean Architecture, Microservices, Monolith).
- **Core Tech Stack:** List primary languages and frameworks (e.g., TypeScript, NestJS, PostgreSQL).

## 🛠 Coding Standards
- **Naming:** Use PascalCase for classes, camelCase for variables/functions.
- **Patterns:** Prefer Functional Programming over OOP where possible. 
- **Error Handling:** Use a Result wrapper pattern rather than throwing exceptions for expected failures.
- **Testing:** Every new feature must include a unit test using Vitest.

## 🚫 Constraints & "Never" Rules
- Never use `any` types; always define an interface.
- Do not use external libraries for simple utility functions (e.g., prefer native `Array` methods over Lodash).
- Never commit secrets or hardcoded API keys.

## 📝 Documentation Requirements
- Use TSDoc for all public-facing methods.
- Every function must have a `@throws` tag if it can fail.
