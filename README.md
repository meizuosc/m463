[M463](http://www.meizu.com)
=================

M463 repo is Linux kernel source code for Meizu **M1 Note / M1** smartphones. With this repo, you can customize the source code and compile a Linux kernel image yourself. Enjoy it!

HOW TO COMPILE
-----------

###1. Download source code###

  <code>git clone https://github.com/meizuosc/m463.git</code>

###2. Compiling###

* For M1 Note
  <br /><code>./build.sh m71</code>
* For M1
  <br /><code>./build.sh m79</code>

  Note:
  + Make sure you have arm cross tool chain, maybe you can download [here](http://www.linaro.org/downloads)
  + If you get a poor cpu in your compiling host, you should use "-j4" or lower instead of "-j8"

Get Help
--------

Checkout our community http://bbs.meizu.cn (in Chinese)
