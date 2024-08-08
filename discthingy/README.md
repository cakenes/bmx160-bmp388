## 1: Yarn, pods, sdk

### For android

```bash
yarn install
```

### Add local.properties file to android/ with:

Windows: `sdk.dir=C:\\Users\\UserName\\AppData\\Local\\Android\\sdk`<br>
Mac: `sdk.dir = /Users/USERNAME/Library/Android/sdk`<br>
Linux: `sdk.dir = /home/USERNAME/Android/Sdk`

### For iOS

```bash
cd ios && bundle install &&  bundle exec pod install
```

###

## 2: Start app

### For Android

```bash
yarn android
```

### For iOS

```bash
yarn ios
```

## 3: Run Metro

```bash
yarn start
```
