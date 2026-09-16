function [ret] = isHex(str)

test = hex2dec(str);
if (isnan(test))
  ret = false;
else 
  ret = true;
endif
