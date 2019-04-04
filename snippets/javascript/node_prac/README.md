# node.js practice

Just practice of node.js and npm.

Reference: https://www.w3schools.com/nodejs/default.asp

To make a package, initialize settings in the directory.
```
npm init
```
This generates `package.json` holding the pacakage dependency information.

Every time a new package is necessary,
```
npm install <package>
```
Then, the package is installed and this dependency information is written into
`package.json`.

To run the code,
```
node <js file>
```
In the case of this repo,
```
node index.js
```

To run the code by `npm start`:
In package.json,
```
"scripts": {
  "start": "node index.js",
  ...
},
```

You can see the results:
```
localhost:8080
```

To choose some feature:
```
localhost:8080/?file=upload
```
etc