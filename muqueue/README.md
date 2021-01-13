# Async functions

Taskeng or **task**let **eng**ine is an engine to run simple tasklets asynchronously. There are internal workqueues served by threads to run user defined functions \(tasklets\) and triggered by events from libev eventloops.

## Example

To run function periodically you can just type

{% tabs %}
{% tab title="C++" %}
```cpp
  fflow::add_periodic<void>(([&](void) -> void { 
    // user code goes here
    // ...
  }), 0.000001, 0.000001);
```
{% endtab %}
{% endtabs %}

To post function to workqueue

{% tabs %}
{% tab title="C++" %}
```cpp
  fflow::post_function<void>([&](void) -> void {
    // user code goes here
    // ...
  });
```
{% endtab %}
{% endtabs %}

[Build instructions and prerequisites](INSTALL.md)

