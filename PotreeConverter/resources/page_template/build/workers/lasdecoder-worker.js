Potree.workers.lasdecoder = new Potree.WorkerManager(atob("CgovL3ZhciBwb2ludEZvcm1hdFJlYWRlcnMgPSB7Ci8vCTA6IGZ1bmN0aW9uKGR2KSB7Ci8vCQlyZXR1cm4gewovLwkJCSJwb3NpdGlvbiI6IFsgZHYuZ2V0SW50MzIoMCwgdHJ1ZSksIGR2LmdldEludDMyKDQsIHRydWUpLCBkdi5nZXRJbnQzMig4LCB0cnVlKV0sCi8vCQkJImludGVuc2l0eSI6IGR2LmdldFVpbnQxNigxMiwgdHJ1ZSksCi8vCQkJImNsYXNzaWZpY2F0aW9uIjogZHYuZ2V0VWludDgoMTYsIHRydWUpCi8vCQl9OwovLwl9LAovLwkxOiBmdW5jdGlvbihkdikgewovLwkJcmV0dXJuIHsKLy8JCQkicG9zaXRpb24iOiBbIGR2LmdldEludDMyKDAsIHRydWUpLCBkdi5nZXRJbnQzMig0LCB0cnVlKSwgZHYuZ2V0SW50MzIoOCwgdHJ1ZSldLAovLwkJCSJpbnRlbnNpdHkiOiBkdi5nZXRVaW50MTYoMTIsIHRydWUpLAovLwkJCSJjbGFzc2lmaWNhdGlvbiI6IGR2LmdldFVpbnQ4KDE2LCB0cnVlKQovLwkJfTsKLy8JfSwKLy8JMjogZnVuY3Rpb24oZHYpIHsKLy8JCXJldHVybiB7Ci8vCQkJInBvc2l0aW9uIjogWyBkdi5nZXRJbnQzMigwLCB0cnVlKSwgZHYuZ2V0SW50MzIoNCwgdHJ1ZSksIGR2LmdldEludDMyKDgsIHRydWUpXSwKLy8JCQkiaW50ZW5zaXR5IjogZHYuZ2V0VWludDE2KDEyLCB0cnVlKSwKLy8JCQkiY2xhc3NpZmljYXRpb24iOiBkdi5nZXRVaW50OCgxNiwgdHJ1ZSksCi8vCQkJImNvbG9yIjogW2R2LmdldFVpbnQxNigyMCwgdHJ1ZSksIGR2LmdldFVpbnQxNigyMiwgdHJ1ZSksIGR2LmdldFVpbnQxNigyNCwgdHJ1ZSldCi8vCQl9OwovLwl9LAovLwkzOiBmdW5jdGlvbihkdikgewovLwkJcmV0dXJuIHsKLy8JCQkicG9zaXRpb24iOiBbIGR2LmdldEludDMyKDAsIHRydWUpLCBkdi5nZXRJbnQzMig0LCB0cnVlKSwgZHYuZ2V0SW50MzIoOCwgdHJ1ZSldLAovLwkJCSJpbnRlbnNpdHkiOiBkdi5nZXRVaW50MTYoMTIsIHRydWUpLAovLwkJCSJjbGFzc2lmaWNhdGlvbiI6IGR2LmdldFVpbnQ4KDE2LCB0cnVlKSwKLy8JCQkiY29sb3IiOiBbZHYuZ2V0VWludDE2KDI4LCB0cnVlKSwgZHYuZ2V0VWludDE2KDMwLCB0cnVlKSwgZHYuZ2V0VWludDE2KDMyLCB0cnVlKV0KLy8JCX07Ci8vCX0KLy99OwovLwkKLy8JCi8vIERlY29kZXMgTEFTIHJlY29yZHMgaW50byBwb2ludHMKLy8KLy92YXIgTEFTRGVjb2RlciA9IGZ1bmN0aW9uKGJ1ZmZlciwgcG9pbnRGb3JtYXRJRCwgcG9pbnRTaXplLCBwb2ludHNDb3VudCwgc2NhbGUsIG9mZnNldCkgewovLwl0aGlzLmFycmF5YiA9IGJ1ZmZlcjsKLy8JdGhpcy5kZWNvZGVyID0gcG9pbnRGb3JtYXRSZWFkZXJzW3BvaW50Rm9ybWF0SURdOwovLwl0aGlzLnBvaW50c0NvdW50ID0gcG9pbnRzQ291bnQ7Ci8vCXRoaXMucG9pbnRTaXplID0gcG9pbnRTaXplOwovLwl0aGlzLnNjYWxlID0gc2NhbGU7Ci8vCXRoaXMub2Zmc2V0ID0gb2Zmc2V0OwovL307Ci8vCi8vTEFTRGVjb2Rlci5wcm90b3R5cGUuZ2V0UG9pbnQgPSBmdW5jdGlvbihpbmRleCkgewovLwlpZiAoaW5kZXggPCAwIHx8IGluZGV4ID49IHRoaXMucG9pbnRzQ291bnQpCi8vCQl0aHJvdyBuZXcgRXJyb3IoIlBvaW50IGluZGV4IG91dCBvZiByYW5nZSIpOwovLwovLwl2YXIgZHYgPSBuZXcgRGF0YVZpZXcodGhpcy5hcnJheWIsIGluZGV4ICogdGhpcy5wb2ludFNpemUsIHRoaXMucG9pbnRTaXplKTsKLy8JcmV0dXJuIHRoaXMuZGVjb2Rlcihkdik7Ci8vfTsKCm9ubWVzc2FnZSA9IGZ1bmN0aW9uKGV2ZW50KXsKCXZhciBidWZmZXIgPSBldmVudC5kYXRhLmJ1ZmZlcjsKCXZhciBudW1Qb2ludHMgPSBldmVudC5kYXRhLm51bVBvaW50czsKCXZhciBwb2ludFNpemUgPSBldmVudC5kYXRhLnBvaW50U2l6ZTsKCXZhciBwb2ludEZvcm1hdElEID0gZXZlbnQuZGF0YS5wb2ludEZvcm1hdElEOwoJdmFyIHNjYWxlID0gZXZlbnQuZGF0YS5zY2FsZTsKCXZhciBvZmZzZXQgPSBldmVudC5kYXRhLm9mZnNldDsKCXZhciBtaW5zID0gZXZlbnQuZGF0YS5taW5zOwoJdmFyIG1heHMgPSBldmVudC5kYXRhLm1heHM7Cgl2YXIgYmJPZmZzZXQgPSBldmVudC5kYXRhLmJiT2Zmc2V0OwoJCgl2YXIgdGVtcCA9IG5ldyBBcnJheUJ1ZmZlcig0KTsKCXZhciB0ZW1wVWludDggPSBuZXcgVWludDhBcnJheSh0ZW1wKTsKCXZhciB0ZW1wVWludDE2ID0gbmV3IFVpbnQxNkFycmF5KHRlbXApOwoJdmFyIHRlbXBGbG9hdDMyID0gbmV3IEZsb2F0MzJBcnJheSh0ZW1wKTsKCXZhciB0ZW1wSW50MzIgPSBuZXcgSW50MzJBcnJheSh0ZW1wKTsKCXZhciBidWZmZXJWaWV3ID0gbmV3IFVpbnQ4QXJyYXkoYnVmZmVyKTsKCQoJdmFyIHBCdWZmID0gbmV3IEFycmF5QnVmZmVyKG51bVBvaW50cyozKjQpOwoJdmFyIGNCdWZmID0gbmV3IEFycmF5QnVmZmVyKG51bVBvaW50cyozKjQpOwoJdmFyIGlCdWZmID0gbmV3IEFycmF5QnVmZmVyKG51bVBvaW50cyo0KTsKCXZhciBjbEJ1ZmYgPSBuZXcgQXJyYXlCdWZmZXIobnVtUG9pbnRzKTsKCXZhciBybkJ1ZmYgPSBuZXcgQXJyYXlCdWZmZXIobnVtUG9pbnRzKTsKCXZhciBwc0J1ZmYgPSBuZXcgQXJyYXlCdWZmZXIobnVtUG9pbnRzICogMik7CgkKCXZhciBwb3NpdGlvbnMgPSBuZXcgRmxvYXQzMkFycmF5KHBCdWZmKTsKCXZhciBjb2xvcnMgPSBuZXcgRmxvYXQzMkFycmF5KGNCdWZmKTsKCXZhciBpbnRlbnNpdGllcyA9IG5ldyBGbG9hdDMyQXJyYXkoaUJ1ZmYpOwoJdmFyIGNsYXNzaWZpY2F0aW9ucyA9IG5ldyBVaW50OEFycmF5KGNsQnVmZik7Cgl2YXIgcmV0dXJuTnVtYmVycyA9IG5ldyBVaW50OEFycmF5KHJuQnVmZik7Cgl2YXIgcG9pbnRTb3VyY2VJRHMgPSBuZXcgVWludDE2QXJyYXkocHNCdWZmKTsKCXZhciB0aWdodEJvdW5kaW5nQm94ID0gewoJCW1pbjogWyBOdW1iZXIuUE9TSVRJVkVfSU5GSU5JVFksIE51bWJlci5QT1NJVElWRV9JTkZJTklUWSwgTnVtYmVyLlBPU0lUSVZFX0lORklOSVRZXSwKCQltYXg6IFsgTnVtYmVyLk5FR0FUSVZFX0lORklOSVRZICwgTnVtYmVyLk5FR0FUSVZFX0lORklOSVRZICwgTnVtYmVyLk5FR0FUSVZFX0lORklOSVRZIF0KCX07CgkKCQoJLy8gdGVtcCBhcnJheXMgc2VlbSB0byBiZSBzaWduaWZpY2FudGx5IGZhc3RlciB0aGFuIERhdGFWaWV3cwoJLy8gYXQgdGhlIG1vbWVudDogaHR0cDovL2pzcGVyZi5jb20vZGF0YXZpZXctdnMtdGVtcG9yYXJ5LWZsb2F0NjRhcnJheQoJZm9yKHZhciBpID0gMDsgaSA8IG51bVBvaW50czsgaSsrKXsKCQoJCS8vIFBPU0lUSU9OCgkJdGVtcFVpbnQ4WzBdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSswXTsKCQl0ZW1wVWludDhbMV0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzFdOwoJCXRlbXBVaW50OFsyXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMl07CgkJdGVtcFVpbnQ4WzNdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSszXTsKCQl2YXIgeCA9IHRlbXBJbnQzMlswXTsKCQkKCQl0ZW1wVWludDhbMF0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzRdOwoJCXRlbXBVaW50OFsxXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrNV07CgkJdGVtcFVpbnQ4WzJdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSs2XTsKCQl0ZW1wVWludDhbM10gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzddOwoJCXZhciB5ID0gdGVtcEludDMyWzBdOwoJCQoJCXRlbXBVaW50OFswXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrOF07CgkJdGVtcFVpbnQ4WzFdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSs5XTsKCQl0ZW1wVWludDhbMl0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzEwXTsKCQl0ZW1wVWludDhbM10gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzExXTsKCQl2YXIgeiA9IHRlbXBJbnQzMlswXTsKCQkKCQlwb3NpdGlvbnNbMyppKzBdID0geCAqIHNjYWxlWzBdICsgb2Zmc2V0WzBdICsgYmJPZmZzZXRbMF07CgkJcG9zaXRpb25zWzMqaSsxXSA9IHkgKiBzY2FsZVsxXSArIG9mZnNldFsxXSArIGJiT2Zmc2V0WzFdOwoJCXBvc2l0aW9uc1szKmkrMl0gPSB6ICogc2NhbGVbMl0gKyBvZmZzZXRbMl0gKyBiYk9mZnNldFsyXTsKCQkKCQl0aWdodEJvdW5kaW5nQm94Lm1pblswXSA9IE1hdGgubWluKHRpZ2h0Qm91bmRpbmdCb3gubWluWzBdLCBwb3NpdGlvbnNbMyppKzBdKTsKCQl0aWdodEJvdW5kaW5nQm94Lm1pblsxXSA9IE1hdGgubWluKHRpZ2h0Qm91bmRpbmdCb3gubWluWzFdLCBwb3NpdGlvbnNbMyppKzFdKTsKCQl0aWdodEJvdW5kaW5nQm94Lm1pblsyXSA9IE1hdGgubWluKHRpZ2h0Qm91bmRpbmdCb3gubWluWzJdLCBwb3NpdGlvbnNbMyppKzJdKTsKCQkKCQl0aWdodEJvdW5kaW5nQm94Lm1heFswXSA9IE1hdGgubWF4KHRpZ2h0Qm91bmRpbmdCb3gubWF4WzBdLCBwb3NpdGlvbnNbMyppKzBdKTsKCQl0aWdodEJvdW5kaW5nQm94Lm1heFsxXSA9IE1hdGgubWF4KHRpZ2h0Qm91bmRpbmdCb3gubWF4WzFdLCBwb3NpdGlvbnNbMyppKzFdKTsKCQl0aWdodEJvdW5kaW5nQm94Lm1heFsyXSA9IE1hdGgubWF4KHRpZ2h0Qm91bmRpbmdCb3gubWF4WzJdLCBwb3NpdGlvbnNbMyppKzJdKTsKCQkKCQkvLyBJTlRFTlNJVFkKCQl0ZW1wVWludDhbMF0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzEyXTsKCQl0ZW1wVWludDhbMV0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzEzXTsKCQl2YXIgaW50ZW5zaXR5ID0gdGVtcFVpbnQxNlswXTsKCQlpbnRlbnNpdGllc1tpXSA9IGludGVuc2l0eTsKCQkKCQkvLyBSRVRVUk4gTlVNQkVSLCBzdG9yZWQgaW4gdGhlIGZpcnN0IDMgYml0cwoJCXZhciByZXR1cm5OdW1iZXIgPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzE0XSAmIDc7CgkJcmV0dXJuTnVtYmVyc1tpXSA9IHJldHVybk51bWJlcjsKCQkKCQkvLyBDTEFTU0lGSUNBVElPTgoJCXZhciBjbGFzc2lmaWNhdGlvbiA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMTVdOwoJCWNsYXNzaWZpY2F0aW9uc1tpXSA9IGNsYXNzaWZpY2F0aW9uOwoJCQoJCS8vIFBPSU5UIFNPVVJDRSBJRAoJCXRlbXBVaW50OFswXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMThdOwoJCXRlbXBVaW50OFsxXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMTldOwoJCXZhciBwb2ludFNvdXJjZUlEID0gdGVtcFVpbnQxNlswXTsKCQlwb2ludFNvdXJjZUlEc1tpXSA9IHBvaW50U291cmNlSUQ7CgkJCgkJLy8gQ09MT1IsIGlmIGF2YWlsYWJsZQoJCWlmKHBvaW50Rm9ybWF0SUQgPT09IDIpewoJCQl0ZW1wVWludDhbMF0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzIwXTsKCQkJdGVtcFVpbnQ4WzFdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSsyMV07CgkJCXZhciByID0gdGVtcFVpbnQxNlswXTsKCQkJCgkJCXRlbXBVaW50OFswXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMjJdOwoJCQl0ZW1wVWludDhbMV0gPSBidWZmZXJWaWV3W2kqcG9pbnRTaXplKzIzXTsKCQkJdmFyIGcgPSB0ZW1wVWludDE2WzBdOwoJCQkKCQkJdGVtcFVpbnQ4WzBdID0gYnVmZmVyVmlld1tpKnBvaW50U2l6ZSsyNF07CgkJCXRlbXBVaW50OFsxXSA9IGJ1ZmZlclZpZXdbaSpwb2ludFNpemUrMjVdOwoJCQl2YXIgYiA9IHRlbXBVaW50MTZbMF07CgkJCQoJCQljb2xvcnNbMyppKzBdID0gciAvIDY1NTM2OwoJCQljb2xvcnNbMyppKzFdID0gZyAvIDY1NTM2OwoJCQljb2xvcnNbMyppKzJdID0gYiAvIDY1NTM2OwoJCX0KCX0KCQoJdmFyIG1lc3NhZ2UgPSB7CgkJcG9zaXRpb246IHBCdWZmLCAKCQljb2xvcjogY0J1ZmYsIAoJCWludGVuc2l0eTogaUJ1ZmYsCgkJY2xhc3NpZmljYXRpb246IGNsQnVmZiwKCQlyZXR1cm5OdW1iZXI6IHJuQnVmZiwKCQlwb2ludFNvdXJjZUlEOiBwc0J1ZmYsCgkJdGlnaHRCb3VuZGluZ0JveDogdGlnaHRCb3VuZGluZ0JveAoJfTsKCQkKCXZhciB0cmFuc2ZlcmFibGVzID0gWwoJCW1lc3NhZ2UucG9zaXRpb24sCgkJbWVzc2FnZS5jb2xvciwgCgkJbWVzc2FnZS5pbnRlbnNpdHksCgkJbWVzc2FnZS5jbGFzc2lmaWNhdGlvbiwKCQltZXNzYWdlLnJldHVybk51bWJlciwKCQltZXNzYWdlLnBvaW50U291cmNlSURdOwoJCQoJcG9zdE1lc3NhZ2UobWVzc2FnZSwgdHJhbnNmZXJhYmxlcyk7Cn0K"));