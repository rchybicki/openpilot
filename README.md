<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 300+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Docs</a>
  <span> · </span>
  <a href="https://docs.comma.ai/contributing/roadmap/">Roadmap</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma 3X</a>
</h3>

Quick start: `bash <(curl -fsSL openpilot.comma.ai)`

[![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml/badge.svg)](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

</div>

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>


Using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma 3/3X, available at [comma.ai/shop](https://comma.ai/shop/comma-3x).
2. **Software:** The setup procedure for the comma 3/3X allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 275+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma 3/3X to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.

------

<div align="center" style="text-align: center;">

<h1>FrogPilot 🐸</h1>

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/FrogAi/FrogPilot)
[![Discord](https://img.shields.io/discord/1137853399715549214?label=Discord)](https://discord.frogpilot.download)
[![Last Updated](https://img.shields.io/badge/Last%20Updated-August%209th%2C%202025-brightgreen)](https://github.com/FrogAi/FrogPilot/releases/latest)
[![Wiki](https://img.shields.io/badge/Wiki-FrogPilot-blue?logo=wiki)](https://frogpilot.wiki.gg/)

</div>

------

**FrogPilot** is a fully open-sourced fork of openpilot, featuring clear and concise commits striving to be a resource for the openpilot developer community. It thrives on contributions from both users and developers, focusing on a collaborative, community-led approach to deliver an advanced openpilot experience for everyone!

openpilot vs **FrogPilot**
------

#### Community
| Feature | openpilot | **FrogPilot** |
|---------|:---------:|:---------:|
| A Welcoming Community | ❌ | ✅ |
| Erich / Primary Moderators / 🦇 | ✅ | ❌ |

#### Core Features
| Feature | openpilot | **FrogPilot** |
|---------|:---------:|:---------:|
| Always On Lateral (Steering) | ❌ | ✅ |
| Blind Spot Integration | ✅ | ✅ |
| Conditional Experimental Mode | ❌ | ✅ |
| Custom Themes | ❌ | ✅ |
| Driver Monitoring | ✅ | ✅ |
| Driving Model Selector | ❌ | ✅ |
| Highly Customizable | ❌ | ✅ |
| Holiday Themes | ❌ | ✅ |
| Speed Limit Support | ❌ | ✅ |
| Weather Detection | ❌ | ✅ |

#### Device & Hardware
| Feature | openpilot | **FrogPilot** |
|---------|:---------:|:---------:|
| Automatic Version Backups | ❌ | ✅ |
| C3 Support | ❌ | ✅ |
| comma Pedal Support | ❌ | ✅ |
| High Quality Recordings | ❌ | ✅ |
| SDSU Support | ❌ | ✅ |

#### Gas/Brake
| Feature | openpilot | **FrogPilot** |
|---------|:---------:|:---------:|
| Adaptive Cruise Control (ACC) | ✅ | ✅ |
| Advanced Live Tuning | ❌ | ✅ |
| Custom Following Distances | ❌ | ✅ |
| Faster Human-Like Acceleration | ❌ | ✅ |
| Smoother Human-Like Braking | ❌ | ✅ |

#### Steering
| Feature | openpilot | **FrogPilot** |
|---------|:---------:|:---------:|
| Advanced Live Tuning | ❌ | ✅ |
| Automatic Lane Changes | ❌ | ✅ |
| Increased Steering Torque* | ❌ | ✅ |
| Lane Centering (LKAS) | ✅ | ✅ |
| Lane Change Assist | ✅ | ✅ |

*Select vehicles only

🌟 Highlight Features
------

### 🚗 Always On Lateral (AOL)
Lane-centering normally works only when openpilot is fully enabled but with **Always On Lateral**, steering assist remains active even when pressing the accelerator or brake!

---

### 🧠 Conditional Experimental Mode (CEM)
**Experimental Mode** enables advanced driving logic, but given it's *experimental* nature, you may not want it active constantly. **Conditional Experimental Mode** automatically switches between **Chill Mode** and **Experimental Mode** based on configured driving conditions!

Examples include:
- Switching to **Experimental Mode** for curves, slower traffic, stop lights, or stop signs
- Returning to **Chill Mode** for stability and predictability

---

### 🎭 Custom Driving Personalities
Not everyone drives the same. With **Custom Driving Personalities**, you control how the car behaves with the following **Personality Profiles**:

- **Traffic:** Optimized for stop-and-go, minimizing delays and gaps
- **Aggressive:** Tighter following distances, faster reactions
- **Standard:** Balanced, all-purpose driving
- **Relaxed:** Smooth maneuvers with larger gaps

Each **Personality Profile** can be fine-tuned to adjust the acceleration, braking, and desired following distance. It’s like teaching your car how *you* drive!

---

### 🎨 Custom Themes
Make openpilot **your own** by customizing the driving screen with:

- Color schemes
- Images
- Sound effects

Enjoy existing **FrogPilot** themes, holiday themes, or even design and share your own creations!

---

### 📏 Speed Limits
**FrogPilot** uses downloaded offline **OpenStreetMaps** data (with optional online **Mapbox** support) to display real-time speed limits. Configurable offsets let you fine-tune adherence, making speed control **effortless** and **natural**!

And lots more! 🚀 From safety enhancements to personalization options, **FrogPilot** continues to evolve with features that put you in control. Check it out today for yourself!

---

🔧 Branches
------
| Branch                     | Install&nbsp;URL          | Description                                            | Recommended&nbsp;For     |
|----------------------------|---------------------------|--------------------------------------------------------|--------------------------|
| FrogPilot                  | frogpilot.download        | The main release branch.                               | Everyone                 |
| FrogPilot&#8209;Staging    | staging.frogpilot.download| Beta branch with upcoming features. Expect bugs!       | Early&nbsp;Adopters      |
| FrogPilot&#8209;Testing    | testing.frogpilot.download| Alpha branch with bleeding-edge features. Breaks often!| Advanced&nbsp;Testers    |
| FrogPilot&#8209;Development| No :)                     | Active development branch. Don't use!                  | **FrogPilot**&nbsp;Developers|
| MAKE&#8209;PRS&#8209;HERE  | No :)                     | Workspace for pull requests. Don't use!                | Contributors             |

🧰 How to Install
------

Easiest way to install **FrogPilot** is via this URL at the installation screen:

```
frogpilot.download
```

**DO NOT** install the **FrogPilot-Development** branch. I'm constantly breaking things on there, so unless you don't want to use openpilot, **NEVER** install it!

![](https://i.imgur.com/LTCqRqB.png)

🐞 Bug reports / Feature Requests
------

If you encounter any issues or bugs while using **FrogPilot**, or if you have any suggestions for new features or improvements, please don't hesitate to post about it on the **[Discord!](https://discord.gg/frogpilot)** I'm always looking for ways to improve the fork and provide a better experience for everyone!

To report a bug or request a new feature, make a post in the [**#bug-reports**](https://discord.com/channels/1137853399715549214/1162100167110053888) or [**#feature-requests**](https://discord.com/channels/1137853399715549214/1160318669839147259) channel respectively on the **FrogPilot Discord**. Please provide as much detail as possible about the issue you're experiencing or the feature you'd like to see added. Photos, videos, log files, or other relevant information are very helpful!

I will do my best to respond to bug reports and feature requests in a timely manner, but please understand that I may not be able to address every request immediately. Your feedback and suggestions are valuable, and I appreciate your help in making **FrogPilot** the best it can be!

📋 Credits
------

* [Aidenir](https://github.com/Aidenir)
* [AlexandreSato](https://github.com/AlexandreSato)
* [cfranyota](https://github.com/cfranyota)
* [cydia2020](https://github.com/cydia2020)
* [dragonpilot-community](https://github.com/dragonpilot-community)
* [ErichMoraga](https://github.com/ErichMoraga)
* [garrettpall](https://github.com/garrettpall)
* [jakethesnake420](https://github.com/jakethesnake420)
* [jyoung8607](https://github.com/jyoung8607)
* [mike8643](https://github.com/mike8643)
* [neokii](https://github.com/neokii)
* [OPGM](https://github.com/opgm)
* [OPKR](https://github.com/openpilotkr)
* [pfeiferj](https://github.com/pfeiferj)
* [realfast](https://github.com/realfast)
* [syncword](https://github.com/syncword)
* [twilsonco](https://github.com/twilsonco)

Star History
------

[![Star History Chart](https://api.star-history.com/svg?repos=FrogAi/FrogPilot&type=Date)](https://www.star-history.com/#FrogAi/FrogPilot&Date)
