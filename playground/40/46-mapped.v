// Benchmark "adder" written by ABC on Thu Jul 18 08:53:21 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n342, new_n344, new_n345,
    new_n348, new_n349, new_n352, new_n353, new_n354, new_n356, new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  inv040aa1d32x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand42aa1n16x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand42aa1d28x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n03x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .out0(new_n104));
  nor002aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n09x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  xorc02aa1n12x5               g012(.a(\a[3] ), .b(\b[2] ), .out0(new_n108));
  nanp03aa1n06x5               g013(.a(new_n104), .b(new_n108), .c(new_n107), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\a[3] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[2] ), .o1(new_n111));
  aoai13aa1n06x5               g016(.a(new_n106), .b(new_n105), .c(new_n110), .d(new_n111), .o1(new_n112));
  nand02aa1n06x5               g017(.a(new_n109), .b(new_n112), .o1(new_n113));
  norp02aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  tech160nm_fixnrc02aa1n05x5   g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  inv030aa1n02x5               g022(.a(new_n117), .o1(new_n118));
  xorc02aa1n12x5               g023(.a(\a[8] ), .b(\b[7] ), .out0(new_n119));
  xorc02aa1n12x5               g024(.a(\a[7] ), .b(\b[6] ), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  nano22aa1n09x5               g026(.a(new_n121), .b(new_n118), .c(new_n116), .out0(new_n122));
  inv040aa1n02x5               g027(.a(new_n114), .o1(new_n123));
  nor042aa1n04x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aob012aa1n06x5               g029(.a(new_n123), .b(new_n124), .c(new_n115), .out0(new_n125));
  orn002aa1n06x5               g030(.a(\a[7] ), .b(\b[6] ), .o(new_n126));
  oaoi03aa1n09x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  aoi013aa1n06x4               g032(.a(new_n127), .b(new_n125), .c(new_n120), .d(new_n119), .o1(new_n128));
  inv000aa1n04x5               g033(.a(new_n128), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[8] ), .b(\a[9] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n129), .c(new_n113), .d(new_n122), .o1(new_n132));
  xorc02aa1n12x5               g037(.a(\a[10] ), .b(\b[9] ), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n132), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g039(.a(\a[10] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[9] ), .o1(new_n136));
  aboi22aa1d24x5               g041(.a(\b[8] ), .b(new_n97), .c(new_n136), .d(new_n135), .out0(new_n137));
  nanp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nor002aa1n16x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  oai112aa1n02x5               g045(.a(new_n140), .b(new_n138), .c(new_n136), .d(new_n135), .o1(new_n141));
  tech160nm_fiaoi012aa1n03p5x5 g046(.a(new_n141), .b(new_n132), .c(new_n137), .o1(new_n142));
  nanb02aa1n06x5               g047(.a(new_n139), .b(new_n138), .out0(new_n143));
  aoi022aa1n02x5               g048(.a(new_n132), .b(new_n137), .c(\b[9] ), .d(\a[10] ), .o1(new_n144));
  aoib12aa1n02x5               g049(.a(new_n142), .b(new_n143), .c(new_n144), .out0(\s[11] ));
  aoai13aa1n02x5               g050(.a(new_n140), .b(new_n141), .c(new_n132), .d(new_n137), .o1(new_n146));
  nor002aa1n03x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  and002aa1n12x5               g052(.a(\b[11] ), .b(\a[12] ), .o(new_n148));
  nor002aa1n03x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  oab012aa1n02x4               g054(.a(new_n139), .b(new_n148), .c(new_n147), .out0(new_n150));
  aboi22aa1n02x7               g055(.a(new_n142), .b(new_n150), .c(new_n146), .d(new_n149), .out0(\s[12] ));
  nona23aa1d18x5               g056(.a(new_n149), .b(new_n133), .c(new_n130), .d(new_n143), .out0(new_n152));
  inv040aa1n02x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n129), .c(new_n113), .d(new_n122), .o1(new_n154));
  oai022aa1n02x5               g059(.a(new_n135), .b(new_n136), .c(\b[10] ), .d(\a[11] ), .o1(new_n155));
  aoi112aa1n03x5               g060(.a(new_n148), .b(new_n147), .c(\a[11] ), .d(\b[10] ), .o1(new_n156));
  nona22aa1n03x5               g061(.a(new_n156), .b(new_n137), .c(new_n155), .out0(new_n157));
  oabi12aa1n02x5               g062(.a(new_n148), .b(new_n139), .c(new_n147), .out0(new_n158));
  nand22aa1n03x5               g063(.a(new_n157), .b(new_n158), .o1(new_n159));
  nanb02aa1n06x5               g064(.a(new_n159), .b(new_n154), .out0(new_n160));
  xorc02aa1n02x5               g065(.a(\a[13] ), .b(\b[12] ), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n161), .b(new_n157), .c(new_n158), .out0(new_n162));
  aoi022aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n154), .d(new_n162), .o1(\s[13] ));
  inv000aa1d42x5               g068(.a(\a[13] ), .o1(new_n164));
  nanb02aa1d24x5               g069(.a(\b[12] ), .b(new_n164), .out0(new_n165));
  xnrc02aa1n02x5               g070(.a(\b[6] ), .b(\a[7] ), .out0(new_n166));
  nona23aa1n09x5               g071(.a(new_n119), .b(new_n116), .c(new_n166), .d(new_n117), .out0(new_n167));
  aoai13aa1n12x5               g072(.a(new_n128), .b(new_n167), .c(new_n109), .d(new_n112), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n161), .b(new_n159), .c(new_n168), .d(new_n153), .o1(new_n169));
  xorc02aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n169), .c(new_n165), .out0(\s[14] ));
  inv000aa1n06x5               g076(.a(\a[14] ), .o1(new_n172));
  xroi22aa1d06x4               g077(.a(new_n164), .b(\b[12] ), .c(new_n172), .d(\b[13] ), .out0(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n159), .c(new_n168), .d(new_n153), .o1(new_n174));
  oaoi03aa1n12x5               g079(.a(\a[14] ), .b(\b[13] ), .c(new_n165), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xorc02aa1n12x5               g081(.a(\a[15] ), .b(\b[14] ), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n176), .out0(\s[15] ));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n175), .c(new_n160), .d(new_n173), .o1(new_n179));
  nor002aa1d24x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n177), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n181), .b(new_n182), .c(new_n174), .d(new_n176), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[16] ), .b(\b[15] ), .out0(new_n184));
  norp02aa1n02x5               g089(.a(new_n184), .b(new_n180), .o1(new_n185));
  aoi022aa1n03x5               g090(.a(new_n183), .b(new_n184), .c(new_n179), .d(new_n185), .o1(\s[16] ));
  nanp02aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  tech160nm_fixnrc02aa1n05x5   g092(.a(\b[15] ), .b(\a[16] ), .out0(new_n188));
  nano22aa1n03x7               g093(.a(new_n188), .b(new_n181), .c(new_n187), .out0(new_n189));
  nano22aa1d15x5               g094(.a(new_n152), .b(new_n173), .c(new_n189), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n129), .c(new_n122), .d(new_n113), .o1(new_n191));
  nand23aa1n03x5               g096(.a(new_n175), .b(new_n177), .c(new_n184), .o1(new_n192));
  oao003aa1n02x5               g097(.a(\a[16] ), .b(\b[15] ), .c(new_n181), .carry(new_n193));
  nanp02aa1n02x5               g098(.a(new_n192), .b(new_n193), .o1(new_n194));
  aoi013aa1n06x4               g099(.a(new_n194), .b(new_n159), .c(new_n173), .d(new_n189), .o1(new_n195));
  nanp02aa1n09x5               g100(.a(new_n191), .b(new_n195), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  nanb03aa1n02x5               g102(.a(new_n197), .b(new_n192), .c(new_n193), .out0(new_n198));
  aoi013aa1n02x4               g103(.a(new_n198), .b(new_n159), .c(new_n173), .d(new_n189), .o1(new_n199));
  aoi022aa1n02x5               g104(.a(new_n196), .b(new_n197), .c(new_n191), .d(new_n199), .o1(\s[17] ));
  inv040aa1d32x5               g105(.a(\a[17] ), .o1(new_n201));
  inv040aa1d28x5               g106(.a(\b[16] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(new_n173), .b(new_n189), .o1(new_n204));
  aobi12aa1n03x5               g109(.a(new_n193), .b(new_n189), .c(new_n175), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n157), .d(new_n158), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n197), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n207));
  nor042aa1n03x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand02aa1d06x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  norb02aa1n03x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n203), .out0(\s[18] ));
  and002aa1n02x5               g116(.a(new_n197), .b(new_n210), .o(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n213));
  aoi013aa1n09x5               g118(.a(new_n208), .b(new_n209), .c(new_n201), .d(new_n202), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nanp02aa1n12x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n09x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n213), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g123(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n203), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n217), .b(new_n220), .c(new_n196), .d(new_n212), .o1(new_n221));
  inv040aa1n08x5               g126(.a(new_n215), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n217), .o1(new_n223));
  aoai13aa1n02x7               g128(.a(new_n222), .b(new_n223), .c(new_n213), .d(new_n214), .o1(new_n224));
  nor002aa1d32x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand42aa1n08x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  inv000aa1d42x5               g132(.a(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[18] ), .o1(new_n229));
  aboi22aa1n03x5               g134(.a(new_n225), .b(new_n226), .c(new_n228), .d(new_n229), .out0(new_n230));
  aoi022aa1n02x7               g135(.a(new_n224), .b(new_n227), .c(new_n221), .d(new_n230), .o1(\s[20] ));
  nano23aa1d15x5               g136(.a(new_n215), .b(new_n225), .c(new_n226), .d(new_n216), .out0(new_n232));
  nand23aa1d12x5               g137(.a(new_n232), .b(new_n197), .c(new_n210), .o1(new_n233));
  aoi012aa1n06x5               g138(.a(new_n233), .b(new_n191), .c(new_n195), .o1(new_n234));
  nona23aa1d18x5               g139(.a(new_n226), .b(new_n216), .c(new_n215), .d(new_n225), .out0(new_n235));
  oaoi03aa1n12x5               g140(.a(\a[20] ), .b(\b[19] ), .c(new_n222), .o1(new_n236));
  inv030aa1n03x5               g141(.a(new_n236), .o1(new_n237));
  oai012aa1d24x5               g142(.a(new_n237), .b(new_n235), .c(new_n214), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[20] ), .b(\a[21] ), .out0(new_n239));
  oabi12aa1n03x5               g144(.a(new_n239), .b(new_n234), .c(new_n238), .out0(new_n240));
  oai112aa1n02x5               g145(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n214), .o1(new_n241));
  oa0012aa1n03x5               g146(.a(new_n240), .b(new_n234), .c(new_n241), .o(\s[21] ));
  inv000aa1d42x5               g147(.a(new_n233), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n238), .o1(new_n245));
  nor042aa1d18x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv000aa1n03x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n02x7               g152(.a(new_n247), .b(new_n239), .c(new_n244), .d(new_n245), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[21] ), .b(\a[22] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n246), .out0(new_n251));
  aoi022aa1n03x5               g156(.a(new_n248), .b(new_n250), .c(new_n240), .d(new_n251), .o1(\s[22] ));
  nor042aa1n06x5               g157(.a(new_n249), .b(new_n239), .o1(new_n253));
  norb02aa1n03x5               g158(.a(new_n253), .b(new_n233), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n255));
  oao003aa1n12x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .carry(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n238), .c(new_n253), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n196), .d(new_n254), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n238), .d(new_n253), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n255), .out0(\s[23] ));
  nor042aa1n09x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n260), .o1(new_n266));
  aoai13aa1n02x7               g171(.a(new_n265), .b(new_n266), .c(new_n255), .d(new_n258), .o1(new_n267));
  tech160nm_fixorc02aa1n03p5x5 g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n264), .o1(new_n269));
  aoi022aa1n02x7               g174(.a(new_n267), .b(new_n268), .c(new_n261), .d(new_n269), .o1(\s[24] ));
  nano32aa1n03x7               g175(.a(new_n233), .b(new_n268), .c(new_n253), .d(new_n260), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n253), .b(new_n236), .c(new_n232), .d(new_n220), .o1(new_n273));
  and002aa1n06x5               g178(.a(new_n268), .b(new_n260), .o(new_n274));
  inv000aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n276));
  aoai13aa1n12x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .d(new_n256), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n277), .c(new_n196), .d(new_n271), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n274), .b(new_n257), .c(new_n238), .d(new_n253), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n278), .o1(new_n281));
  and003aa1n02x5               g186(.a(new_n280), .b(new_n281), .c(new_n276), .o(new_n282));
  aobi12aa1n02x7               g187(.a(new_n279), .b(new_n282), .c(new_n272), .out0(\s[25] ));
  inv000aa1n02x5               g188(.a(new_n277), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoai13aa1n02x7               g191(.a(new_n286), .b(new_n281), .c(new_n272), .d(new_n284), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n285), .o1(new_n289));
  aoi022aa1n03x5               g194(.a(new_n287), .b(new_n288), .c(new_n279), .d(new_n289), .o1(\s[26] ));
  and002aa1n06x5               g195(.a(new_n288), .b(new_n278), .o(new_n291));
  inv020aa1n03x5               g196(.a(new_n291), .o1(new_n292));
  nano23aa1n06x5               g197(.a(new_n292), .b(new_n233), .c(new_n274), .d(new_n253), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n206), .c(new_n168), .d(new_n190), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(\b[25] ), .b(\a[26] ), .o1(new_n295));
  oai022aa1n02x5               g200(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n296), .b(new_n295), .o1(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n292), .c(new_n280), .d(new_n276), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n196), .d(new_n293), .o1(new_n300));
  aoi122aa1n03x5               g205(.a(new_n299), .b(new_n295), .c(new_n296), .d(new_n277), .e(new_n291), .o1(new_n301));
  aobi12aa1n03x7               g206(.a(new_n300), .b(new_n301), .c(new_n294), .out0(\s[27] ));
  aoi022aa1n06x5               g207(.a(new_n277), .b(new_n291), .c(new_n295), .d(new_n296), .o1(new_n303));
  nor042aa1d18x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  inv040aa1n08x5               g209(.a(new_n304), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n299), .o1(new_n306));
  aoai13aa1n02x7               g211(.a(new_n305), .b(new_n306), .c(new_n303), .d(new_n294), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n304), .o1(new_n309));
  aoi022aa1n02x7               g214(.a(new_n307), .b(new_n308), .c(new_n300), .d(new_n309), .o1(\s[28] ));
  and002aa1n02x5               g215(.a(new_n308), .b(new_n299), .o(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n298), .c(new_n196), .d(new_n293), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  oao003aa1n03x5               g218(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n313), .c(new_n303), .d(new_n294), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n314), .b(new_n316), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g224(.a(new_n306), .b(new_n308), .c(new_n316), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n298), .c(new_n196), .d(new_n293), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  tech160nm_fioaoi03aa1n03p5x5 g227(.a(\a[29] ), .b(\b[28] ), .c(new_n314), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n323), .o1(new_n324));
  aoai13aa1n02x7               g229(.a(new_n324), .b(new_n322), .c(new_n303), .d(new_n294), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  and002aa1n02x5               g231(.a(\b[28] ), .b(\a[29] ), .o(new_n327));
  oabi12aa1n02x5               g232(.a(new_n326), .b(\a[29] ), .c(\b[28] ), .out0(new_n328));
  oab012aa1n02x4               g233(.a(new_n328), .b(new_n314), .c(new_n327), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n325), .b(new_n326), .c(new_n321), .d(new_n329), .o1(\s[30] ));
  nano32aa1n06x5               g235(.a(new_n306), .b(new_n326), .c(new_n308), .d(new_n316), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n298), .c(new_n196), .d(new_n293), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  inv000aa1d42x5               g238(.a(\a[30] ), .o1(new_n334));
  inv000aa1d42x5               g239(.a(\b[29] ), .o1(new_n335));
  oabi12aa1n02x5               g240(.a(new_n333), .b(\a[30] ), .c(\b[29] ), .out0(new_n336));
  oaoi13aa1n02x5               g241(.a(new_n336), .b(new_n323), .c(new_n334), .d(new_n335), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n331), .o1(new_n338));
  oaoi03aa1n03x5               g243(.a(new_n334), .b(new_n335), .c(new_n323), .o1(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n338), .c(new_n303), .d(new_n294), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n340), .b(new_n333), .c(new_n332), .d(new_n337), .o1(\s[31] ));
  nanp03aa1n02x5               g246(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n108), .b(new_n342), .c(new_n101), .out0(\s[3] ));
  obai22aa1n02x7               g248(.a(new_n106), .b(new_n105), .c(\a[3] ), .d(\b[2] ), .out0(new_n344));
  aoi012aa1n02x5               g249(.a(new_n344), .b(new_n104), .c(new_n108), .o1(new_n345));
  oaoi13aa1n02x5               g250(.a(new_n345), .b(new_n113), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g251(.a(new_n118), .b(new_n109), .c(new_n112), .out0(\s[5] ));
  aoai13aa1n06x5               g252(.a(new_n116), .b(new_n124), .c(new_n113), .d(new_n118), .o1(new_n348));
  aoi112aa1n02x5               g253(.a(new_n124), .b(new_n116), .c(new_n113), .d(new_n118), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n348), .b(new_n349), .out0(\s[6] ));
  xnbna2aa1n03x5               g255(.a(new_n120), .b(new_n348), .c(new_n123), .out0(\s[7] ));
  aob012aa1n02x5               g256(.a(new_n120), .b(new_n348), .c(new_n123), .out0(new_n352));
  aoai13aa1n02x5               g257(.a(new_n126), .b(new_n166), .c(new_n348), .d(new_n123), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n126), .b(new_n119), .out0(new_n354));
  aoi022aa1n03x5               g259(.a(new_n353), .b(new_n119), .c(new_n352), .d(new_n354), .o1(\s[8] ));
  nanp02aa1n02x5               g260(.a(new_n122), .b(new_n113), .o1(new_n356));
  aoi113aa1n02x5               g261(.a(new_n131), .b(new_n127), .c(new_n125), .d(new_n120), .e(new_n119), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n168), .b(new_n131), .c(new_n356), .d(new_n357), .o1(\s[9] ));
endmodule


