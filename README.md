# grid_mapping

[message_filters TimeSynchronizer](https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/)
[message_filters ApproximateTime](https://qiita.com/porizou1/items/edd69a84bc8f47db42c2)

### 使用message_filters注意点

- 1.回调函数一定放在public或protected下面，不能在private下面，否则回调函数不调用(本package中指的是livegmapping()方法)
