# Local-Global Hybrid Stabilization via Quadratic Lyapunov Function and Transport PDE

## System Description

Consider the system:

$$\dot{x} = f(x) + G(x)u, \quad x \in \mathbb{R}^n, \quad u \in \mathbb{R}^m$$

where $f \in C^1(\mathbb{R}^n, \mathbb{R}^n)$, $G \in C^1(\mathbb{R}^n, \mathbb{R}^{n \times m})$, and $f(0) = 0$.

The objective is to stabilize the equilibrium $x = 0$.

Set:
$$A := Df(0), \quad B := G(0)$$

## Assumption 1: Stabilizability

The pair $(A, B)$ is stabilizable. That is, there exists a matrix $K \in \mathbb{R}^{m \times n}$ such that:

$$A_{cl} := A + BK$$

is a Hurwitz matrix.

## Local Control Law

In the inner region, we use:

$$u_{loc}(x) = Kx$$

## Lemma 1: Local Stability

Suppose Assumption 1 holds. Then there exist a symmetric positive definite matrix $P = P^\top > 0$, a number $a > 0$, and a number $c > 0$ such that for the function:

$$W(x) := x^\top P x$$

the estimate holds:

$$L_{f+Gu_{loc}} W(x) \leq -c W(x), \quad 0 < W(x) \leq a$$

Define the boundary set:

$$\Sigma_a := \{x \in \mathbb{R}^n : W(x) = a\}$$

and the inner region:

$$\Omega_a := \{x \in \mathbb{R}^n : W(x) \leq a\}$$

Then:
- $\Omega_a$ is positively invariant for $\dot{x} = f(x) + G(x)u_{loc}(x)$
- The equilibrium $x = 0$ is asymptotically stable in $\Omega_a$

### Proof Sketch

Since $A_{cl}$ is Hurwitz, for any $Q = Q^\top > 0$, the Lyapunov equation:

$$A_{cl}^\top P + P A_{cl} = -Q$$

has a unique solution $P = P^\top > 0$. Fix $Q = I$.

Set $F_{loc}(x) := f(x) + G(x)Kx$. Then $F_{loc}(x) = A_{cl}x + o(\|x\|)$ as $x \to 0$.

Therefore:
$$L_{F_{loc}} W(x) = \nabla W(x)^\top F_{loc}(x) = 2x^\top P A_{cl} x + o(\|x\|^2) = x^\top(A_{cl}^\top P + P A_{cl})x + o(\|x\|^2) = -\|x\|^2 + o(\|x\|^2)$$

There exist $r > 0$ and $\mu > 0$ such that for $0 < \|x\| < r$:

$$L_{F_{loc}} W(x) \leq -\mu \|x\|^2$$

Since $P > 0$, there exist $\lambda_{min}, \lambda_{max} > 0$ such that:

$$\lambda_{min} \|x\|^2 \leq W(x) \leq \lambda_{max} \|x\|^2$$

Therefore, we can choose $a > 0$ so that $W(x) \leq a$ implies $\|x\| < r$, and then:

$$L_{F_{loc}} W(x) \leq -\frac{\mu}{\lambda_{max}} W(x) =: -c W(x), \quad 0 < W(x) \leq a$$

On the boundary $\Sigma_a$, we have $L_{F_{loc}} W(x) \leq -ca < 0$, so the vector field points inward $\Omega_a$, and $\Omega_a$ is positively invariant. Asymptotic stability follows from the direct Lyapunov method.

## Outer Control Region

The outer region is defined as:

$$D_a := \mathbb{R}^n \setminus \Omega_a = \{x \in \mathbb{R}^n : W(x) > a\}$$

In the outer region, introduce the function:

$$T(x) := \log\left(\frac{W(x)}{a}\right), \quad x \in D_a$$

Then:
$$T \in C^\infty(D_a), \quad T > 0 \text{ on } D_a, \quad T = 0 \text{ on } \Sigma_a$$

$$\nabla T(x) = \frac{\nabla W(x)}{W(x)}$$

## Assumption 2: Control Authority

For all $x \in D_a$, the condition:

$$G(x)^\top \nabla W(x) \neq 0$$

holds.

## Outer Control Law

The outer control is defined by:

$$u_{ext}(x) := -\frac{W(x) + \nabla W(x)^\top f(x)}{\|G(x)^\top \nabla W(x)\|^2} G(x)^\top \nabla W(x), \quad x \in D_a$$

## Lemma 2: Outer Control Property

Suppose Assumption 2 holds. Then for all $x \in D_a$:

$$\nabla T(x)^\top (f(x) + G(x)u_{ext}(x)) = -1$$

Equivalently:

$$L_{f+Gu_{ext}} W(x) = -W(x), \quad x \in D_a$$

### Proof

Set $b(x) := G(x)^\top \nabla W(x) \in \mathbb{R}^m$. Then:

$$u_{ext}(x) = -\frac{W(x) + \nabla W(x)^\top f(x)}{\|b(x)\|^2} b(x)$$

Therefore:
$$\nabla W(x)^\top G(x)u_{ext}(x) = -W(x) - \nabla W(x)^\top f(x)$$

Hence:
$$\nabla W(x)^\top (f(x) + G(x)u_{ext}(x)) = -W(x)$$

Using the relation:
$$\nabla T(x)^\top (f(x) + G(x)u_{ext}(x)) = \frac{1}{W(x)}(-W(x)) = -1 \quad \checkmark$$

## Lemma 3: Exponential Convergence

Let $x(t)$ be a solution of:

$$\dot{x} = f(x) + G(x)u_{ext}(x), \quad x(0) = x_0 \in D_a$$

Then, as long as the trajectory remains in $D_a$:

$$W(x(t)) = W(x_0)e^{-t}$$

In particular, the trajectory reaches $\Sigma_a$ in finite time:

$$\tau_{in}(x_0) = \log\left(\frac{W(x_0)}{a}\right)$$

### Proof

By Lemma 2:
$$\frac{d}{dt}W(x(t)) = -W(x(t))$$

Therefore:
$$W(x(t)) = W(x_0)e^{-t}$$

The first hitting time of $\Sigma_a$ satisfies:
$$W(x_0)e^{-\tau} = a \quad \Rightarrow \quad \tau = \log\left(\frac{W(x_0)}{a}\right)$$

## Lemma 4: Solution Existence

For every $x_0 \in D_a$, the outer trajectory exists on the whole interval $[0, \tau_{in}(x_0)]$.

### Proof

From the exponential formula, for $0 \leq t \leq \tau_{in}(x_0)$:

$$a \leq W(x(t)) \leq W(x_0)$$

The set $\{x \in \mathbb{R}^n : a \leq W(x) \leq W(x_0)\}$ is compact. Since $f$, $G$, and $u_{ext}$ are locally Lipschitz on $D_a$, finite-time blow-up is impossible.

## Hybrid Control Law

The control is applied according to:

$$u(t) = \begin{cases}
u_{ext}(x(t)), & 0 \leq t < \tau_{in}(x_0), \quad x_0 \in D_a \\
u_{loc}(x(t)), & t \geq \tau_{in}(x_0), \quad x_0 \in D_a \\
u_{loc}(x(t)), & t \geq 0, \quad x_0 \in \Omega_a
\end{cases}$$

## Theorem 1: Global Asymptotic Stability

Consider system (1). Assume:

1. Assumption 1 holds
2. $P > 0$, $a > 0$, $c > 0$ are chosen as in Lemma 1
3. Assumption 2 holds
4. The outer control is given by the formula above
5. The control is applied according to the hybrid law

Then the equilibrium $x = 0$ is **globally asymptotically stable**.

### Proof

**Case 1:** If $x_0 \in \Omega_a$, then by Lemma 1, $\Omega_a$ is positively invariant and asymptotically stable. Thus $x(t) \to 0$ as $t \to \infty$.

**Case 2:** If $x_0 \in D_a$, then by Lemma 4, the outer solution exists on $[0, \tau_{in}(x_0)]$, and by Lemma 3:

$$x(\tau_{in}(x_0)) \in \Sigma_a \subset \Omega_a$$

After entering $\Omega_a$, the local stabilizer $u_{loc}$ is applied. Since $\Omega_a$ is positively invariant, the trajectory remains in $\Omega_a$ and converges to zero.

**Stability:** There exists $\delta > 0$ such that:

$$\|x_0\| < \delta \quad \Rightarrow \quad W(x_0) < a$$

So $x_0 \in \Omega_a$ from the start. By local asymptotic stability, the origin is stable.

Therefore, the origin is **stable and globally attractive**, hence **globally asymptotically stable**. ∎

---

## Application: Two-Tank Hydraulic System

Consider two interconnected tanks with level deviations:

$$x = \begin{pmatrix} h_1 \\ h_2 \end{pmatrix} \in \mathbb{R}^2$$

and control inflows:

$$u = \begin{pmatrix} u_1 \\ u_2 \end{pmatrix} \in \mathbb{R}^2$$

The system dynamics are:

$$\dot{x} = f(x) + u$$

where:

$$f(x) = \begin{pmatrix}
\alpha \dfrac{h_1^2}{r^2 + h_1^2} h_1 + \kappa(h_2 - h_1) \\[0.3cm]
\alpha \dfrac{h_2^2}{r^2 + h_2^2} h_2 + \kappa(h_1 - h_2)
\end{pmatrix}$$

with $\alpha > 0$, $\kappa > 0$, $r > 0$.

The terms $\kappa(h_2 - h_1)$ and $\kappa(h_1 - h_2)$ describe flow exchange between tanks, while the nonlinear terms model a destabilizing effect that grows with level deviations.

### Local Stabilizer

The linearization at the origin is:

$$A = \begin{pmatrix} -\kappa & \kappa \\ \kappa & -\kappa \end{pmatrix}, \quad B = I_2$$

Using $K = -kI_2$ with $k > 0$:

$$A_{cl} = \begin{pmatrix} -(\kappa + k) & \kappa \\ \kappa & -(\kappa + k) \end{pmatrix}$$

with eigenvalues $-k$ and $-(2\kappa + k)$, so $A_{cl}$ is Hurwitz. However, on the invariant set $h_1 = h_2 = s$:

$$\dot{s} = \alpha \frac{s^2}{r^2 + s^2} s - ks$$

For large $|s|$, $\dot{s} \approx (\alpha - k)s$. If $\alpha > k$, trajectories diverge—local control alone is not global.

### Lyapunov Function

$$W(x) = h_1^2 + h_2^2$$

with:
$$\nabla W(x) = 2\begin{pmatrix} h_1 \\ h_2 \end{pmatrix}$$

Since $G(x) = I_2$, we have $G(x)^\top \nabla W(x) = \nabla W(x) \neq 0$ for all $x \neq 0$.

### Outer Control

In $D_a = \{x : W(x) > a\}$, we have $\|\nabla W(x)\|^2 = 4W(x)$ and:

$$\nabla W(x)^\top f(x) = 2\alpha \frac{h_1^4}{r^2 + h_1^2} + 2\alpha \frac{h_2^4}{r^2 + h_2^2} - 2\kappa(h_1 - h_2)^2$$

Therefore:

$$u_{ext}(x) = -\frac{1}{2W(x)} \left( W(x) + 2\alpha \frac{h_1^4}{r^2 + h_1^2} + 2\alpha \frac{h_2^4}{r^2 + h_2^2} - 2\kappa(h_1 - h_2)^2 \right) \begin{pmatrix} h_1 \\ h_2 \end{pmatrix}, \quad x \neq 0$$

Then: $L_{f+u_{ext}} W(x) = -W(x)$ and consequently: $W(x(t)) = W(x_0)e^{-t}$

### Overall Control Strategy

For $x_0 \in D_a$, use $u_{ext}$ until $W(x(t)) = a$ (in finite time $\tau_{in}(x_0) = \log(W(x_0)/a)$), then switch to $u_{loc}$ for stabilization in $\Omega_a$.

For $x_0 \in \Omega_a$, use $u_{loc}$ directly.

---

## Remark 1: Alternative Formulation

The outer law can be rewritten in terms of $T$. Since $\nabla T(x) = \frac{\nabla W(x)}{W(x)}$, introducing $\beta(x) := G(x)^\top \nabla T(x)$:

$$u_{ext}(x) = -\frac{1 + \nabla T(x)^\top f(x)}{\|\beta(x)\|^2} \beta(x)$$

Then automatically:
$$\nabla T(x)^\top (f(x) + G(x)u_{ext}(x)) = -1$$

For $T(x) = \log\frac{W(x)}{a}$, this is equivalent to $\dot{W} = -W$ in the outer region.
