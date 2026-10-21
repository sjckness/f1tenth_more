// One-axis joystick (gas: vertical, steering: horizontal). Pointer Events, so each
// stick follows its own finger: two thumbs at once work. Springs back to 0 on release.
import { useRef, useState } from 'preact/hooks';
import { capture } from './pointer';

export function Joystick({ axis, label, disabled, glow, reverse, onChange, testid }: {
  axis: 'v' | 'h';
  label: string;
  disabled?: boolean;
  glow?: boolean;                       // the dead-man is held: green ring
  reverse?: boolean;                    // gas: the lower half is reverse, ends AVANTI / RETRO (amber)
  onChange: (value: number, active: boolean) => void;
  testid?: string;
}) {
  const ref = useRef<HTMLDivElement>(null);
  const pointer = useRef<number | null>(null);
  const [value, setValue] = useState(0);
  const long = 284;
  const short = 104;
  const knob = 80;
  const travel = (long - knob) / 2 - 12;

  const update = (e: PointerEvent, active: boolean) => {
    const el = ref.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    let v: number;
    if (axis === 'v') v = -((e.clientY - (r.top + r.height / 2)) / travel);
    else v = (e.clientX - (r.left + r.width / 2)) / travel;
    v = Math.max(-1, Math.min(1, v));
    setValue(v);
    onChange(v, active);
  };
  const down = (e: PointerEvent) => {
    if (disabled || pointer.current !== null) return;
    e.preventDefault();
    pointer.current = e.pointerId;
    capture(e);
    update(e, true);
  };
  const move = (e: PointerEvent) => {
    if (pointer.current !== e.pointerId) return;
    e.preventDefault();
    update(e, true);
  };
  const up = (e: PointerEvent) => {
    if (pointer.current !== e.pointerId) return;
    pointer.current = null;
    setValue(0);
    onChange(0, false);
  };
  const vertical = axis === 'v';
  const w = vertical ? short : long;
  const h = vertical ? long : short;
  const off = value * travel;
  const kx = vertical ? (w - knob) / 2 : (w - knob) / 2 + off;
  const ky = vertical ? (h - knob) / 2 - off : (h - knob) / 2;
  // The pad maps where the thumb is: a label at an end sits at full travel that way.
  const endV = { position: 'absolute', left: 0, right: 0, textAlign: 'center', fontSize: '11px',
    color: 'var(--muted)', letterSpacing: '0.1em', pointerEvents: 'none' } as const;
  return (
    <div ref={ref} role="slider" aria-label={label} aria-orientation={vertical ? 'vertical' : 'horizontal'}
      aria-valuemin={-100} aria-valuemax={100} aria-valuenow={Math.round(value * 100)} tabIndex={0}
      aria-disabled={disabled} data-testid={testid}
      onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}
      onContextMenu={(e) => e.preventDefault()}
      style={{
        position: 'relative', width: `${w}px`, height: `${h}px`, borderRadius: '52px', flexShrink: 0,
        background: 'var(--surface)', touchAction: 'none', opacity: disabled ? 0.4 : 1,
        border: glow ? '2px solid var(--green)' : '1px solid var(--border)',
        boxShadow: glow ? '0 0 0 5px rgba(52,210,123,0.15)' : 'none',
      }}>
      {vertical
        ? <div style={{ position: 'absolute', left: '20px', right: '20px', top: `${h / 2 - 1}px`, height: '1.5px', background: '#3A404B' }} />
        : <div style={{ position: 'absolute', top: '20px', bottom: '20px', left: `${w / 2 - 1}px`, width: '1.5px', background: '#3A404B' }} />}
      <div style={{ position: 'absolute', left: `${kx}px`, top: `${ky}px`, width: `${knob}px`, height: `${knob}px`,
        borderRadius: '50%', background: 'var(--text)', boxShadow: '0 6px 18px rgba(0,0,0,0.5)',
        pointerEvents: 'none' }} />
      {vertical
        ? reverse
          ? <>
            <span class="mono" style={{ ...endV, top: '14px' }}>AVANTI</span>
            <span class="mono" data-testid={testid && `${testid}-reverse-end`}
              style={{ ...endV, bottom: '14px', color: 'var(--amber)', fontWeight: 700 }}>RETRO</span></>
          : <span class="mono" style={{ ...endV, bottom: '14px' }}>GAS</span>
        : <>
          <span class="mono" style={{ position: 'absolute', left: '18px', top: `${h / 2 - 8}px`, fontSize: '11px', color: 'var(--muted)', pointerEvents: 'none' }}>SX</span>
          <span class="mono" style={{ position: 'absolute', right: '18px', top: `${h / 2 - 8}px`, fontSize: '11px', color: 'var(--muted)', pointerEvents: 'none' }}>DX</span></>}
    </div>
  );
}
