// setPointerCapture throws for a pointer the browser does not know (synthetic events in
// tests, a touch already gone): capture when possible, never let it break the gesture.
export function capture(e: PointerEvent): void {
  try {
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
  } catch { /* the element still gets the events it is under */ }
}
