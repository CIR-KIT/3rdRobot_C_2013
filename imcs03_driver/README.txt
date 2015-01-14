Linux kernel 3.2 用 iMCs03 USB ドライバ 2012年9月14日版
株式会社イクシスリサーチ

***ファイルについて***

Linux kernel 2.6 用 iMCs03 USB ドライバソース usc2.6ディレクトリ には以下のファ
イルが含まれています。

-usc3.2
  iMCs03ドライバインストールマニュアル_Linux kernel3.2用_Ver1.0.pdf
  README.txt			// 現在開いているファイル
  Makefile			// サンプルアプリ用 Makefile
  h8test.c			// サンプルアプリ1
  uread.c			// サンプルアプリ2
  uwrite.c			// サンプルアプリ3
--driver			// ドライバ用ディレクトリ
    Makefile			// ビルド用の Makefile
    usc.c			// ドライバ本体
    usc.h			// ioctl 用のヘッダ
    usensorc.h			// read/write するデータを定義するヘッダ

***ビルドおよび動作環境***
Ubuntu12.04で、ビルドおよび動作確認をしています。

ビルドには、uname -r でわかる現在動作中の kernel に対応する、
linux-source パッケージをインストールしておく必要があります。
$ uname -r
3.2.0-29-generic-pae

となるような環境では、linux-source-3.2.0 パッケージをあらかじめインストールしておいてください。
$ sudo apt-get install linux-source-3.2.0

カーネルソースをダウンロード後、以下を実施し、必要なファイル類の解凍を行います。
$ cd /usr/src
$ sudo tar xf linux-source-3.2.0.tar.bz2

/boot以下の現在のカーネル環境の.configを取得します。
$ cd linux-source-3.2.0
$ sudo cp /boot/confing-3.2.0-30-generic-pae .config

その後、makeを実施します。
$ sudo make oldconfig
$ sudo make


***ビルド方法***

ビルドはドライバのソースを展開したディレクトリに移動し、
$ make

とします。うまくコンパイルが成功するとドライバモジュールファイル
usc.ko が作成されます。

適切な kernel-devel パッケージがインストールされていない場合、以下のように表示されて make が失敗します。

$ make
make -C /lib/modules/2.6.15-1.2054_FC5/build M=/disk/tonki/d50/proj/ixs-usb/work/mrt/usc-2.6 modules
make: *** /lib/modules/2.6.15-1.2054_FC5/build: No such file or directory.  Stop.
make: *** [default] Error 2


***動作確認***

以下のプログラムについて動作確認しています。

h8test
uread
uwrite

環境は Ubuntu12.04です。

手順は
$ sudo insmod usc.ko
としてドライバをロードしたのちに、デバイスを USB コネクタに差します。

/dev/usc0が作られることを確認します。
$ ls -l /dev/usc*

アクセス権を変更します。
$ sudo chmod 777 /dev/usc0

上記のプログラムを順次起動しています。
