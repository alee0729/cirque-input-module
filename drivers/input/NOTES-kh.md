Add the following somewhere:

```
		input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
	} else {
		input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
```

note:

* device is operated in relative mode

Use https://zmk.dev/docs/keymaps/input-processors/temp-layer to somehow force mouse layer
from /workspaces/zmk/app/src/pointing/input_processor_temp_layer.c
