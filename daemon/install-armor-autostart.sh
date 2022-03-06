#!/usr/bin/sh
apt intall xvfb # FIXME: 直接不开启GUI,而不是用xvfb-run
mkdir -p /usr/local/lib/systemd/system
cp daemon/armor-autostart.service /usr/local/lib/systemd/system
systemctl daemon-reload
systemctl reenable armor-autostart
