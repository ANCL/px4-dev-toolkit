# Mathematical foundations

The rotation $R$ maps the forward-right-down (FRD) body frame into the north-east-down (NED) local frame. Gravity is $g e_3$ and thrust acts along $-R e_3$.

The implemented translation is $a_{\mathrm{cmd}}=a_d-(k_x/m)e_x-(k_v/m)e_v$, with physical force $A=m(a_{\mathrm{cmd}}-g e_3)$. Read the [SE(3) introduction](../se3.md) for command choices.
