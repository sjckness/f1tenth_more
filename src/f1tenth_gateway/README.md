# f1tenth_gateway

The gateway for the F1Tenth Lab Control PWA. Installation, `tailscale serve`,
the token and the iPhone are described in the full README (written with the
frontend, Phase 4).

Run it on linus against the pretend car:

```bash
cd src/f1tenth_gateway
F1T_GATEWAY_TOKEN=$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))') \
  bin/f1tenth_gateway --mock
```
