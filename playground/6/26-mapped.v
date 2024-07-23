// Benchmark "adder" written by ABC on Wed Jul 17 15:13:17 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n326, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n09x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nanp02aa1n03x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n103), .b(new_n106), .c(new_n105), .d(new_n104), .out0(new_n107));
  aoi012aa1n02x7               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n04x7               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  tech160nm_fixnrc02aa1n02p5x5 g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .out0(new_n111));
  nor042aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n116), .b(new_n111), .c(new_n110), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  oaib12aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(\a[6] ), .out0(new_n122));
  oai012aa1n04x7               g027(.a(new_n119), .b(new_n116), .c(new_n122), .o1(new_n123));
  inv000aa1n03x5               g028(.a(new_n123), .o1(new_n124));
  nand42aa1d28x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n98), .b(new_n126), .c(new_n118), .d(new_n124), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1n02x5               g033(.a(new_n102), .o1(new_n129));
  nano23aa1n02x4               g034(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n103), .out0(new_n130));
  aobi12aa1n02x5               g035(.a(new_n108), .b(new_n130), .c(new_n129), .out0(new_n131));
  nano23aa1n02x4               g036(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n111), .c(new_n110), .out0(new_n133));
  oai012aa1n06x5               g038(.a(new_n124), .b(new_n131), .c(new_n133), .o1(new_n134));
  nor042aa1d18x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n97), .c(new_n134), .d(new_n125), .o1(new_n138));
  inv000aa1d42x5               g043(.a(\b[10] ), .o1(new_n139));
  nanb02aa1d36x5               g044(.a(\a[11] ), .b(new_n139), .out0(new_n140));
  nand22aa1n12x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n16x5               g046(.a(new_n140), .b(new_n141), .o1(new_n142));
  inv040aa1n08x5               g047(.a(new_n142), .o1(new_n143));
  oai012aa1d24x5               g048(.a(new_n136), .b(new_n135), .c(new_n97), .o1(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n143), .b(new_n138), .c(new_n144), .out0(\s[11] ));
  inv000aa1d42x5               g050(.a(new_n144), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n143), .b(new_n146), .c(new_n127), .d(new_n137), .o1(new_n147));
  xnrc02aa1n12x5               g052(.a(\b[11] ), .b(\a[12] ), .out0(new_n148));
  nanp03aa1n02x5               g053(.a(new_n147), .b(new_n140), .c(new_n148), .o1(new_n149));
  tech160nm_fiaoi012aa1n02p5x5 g054(.a(new_n148), .b(new_n147), .c(new_n140), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(\s[12] ));
  xorc02aa1n12x5               g056(.a(\a[12] ), .b(\b[11] ), .out0(new_n152));
  nano23aa1n09x5               g057(.a(new_n97), .b(new_n135), .c(new_n136), .d(new_n125), .out0(new_n153));
  nand23aa1d12x5               g058(.a(new_n153), .b(new_n143), .c(new_n152), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n03x5               g060(.a(new_n155), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n156));
  oaoi03aa1n03x5               g061(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n157));
  inv030aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  oai013aa1d12x5               g063(.a(new_n158), .b(new_n148), .c(new_n144), .d(new_n142), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n06x4               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n156), .c(new_n160), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(new_n161), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n163), .b(new_n159), .c(new_n134), .d(new_n155), .o1(new_n166));
  norp02aa1n04x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n03x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n165), .out0(\s[14] ));
  oai012aa1n02x5               g075(.a(new_n168), .b(new_n167), .c(new_n161), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n168), .b(new_n162), .c(new_n161), .d(new_n167), .out0(new_n172));
  aoai13aa1n02x7               g077(.a(new_n171), .b(new_n172), .c(new_n156), .d(new_n160), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nor042aa1n04x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n09x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n175), .b(new_n180), .c(new_n173), .d(new_n176), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n175), .c(new_n173), .d(new_n176), .o1(new_n182));
  norb02aa1n02x7               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  xorc02aa1n12x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  nano23aa1n03x7               g089(.a(new_n175), .b(new_n177), .c(new_n178), .d(new_n176), .out0(new_n185));
  nano32aa1d12x5               g090(.a(new_n154), .b(new_n185), .c(new_n163), .d(new_n169), .out0(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n175), .b(new_n176), .out0(new_n188));
  nor043aa1n03x5               g093(.a(new_n172), .b(new_n188), .c(new_n179), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nona23aa1n02x4               g095(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n177), .out0(new_n191));
  oai022aa1n02x5               g096(.a(new_n191), .b(new_n171), .c(\b[15] ), .d(\a[16] ), .o1(new_n192));
  aoi112aa1n09x5               g097(.a(new_n192), .b(new_n190), .c(new_n159), .d(new_n189), .o1(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n184), .b(new_n187), .c(new_n193), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  nanp02aa1n06x5               g101(.a(new_n187), .b(new_n193), .o1(new_n197));
  tech160nm_fioaoi03aa1n03p5x5 g102(.a(new_n195), .b(new_n196), .c(new_n197), .o1(new_n198));
  nor022aa1n04x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand02aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanb02aa1n06x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  tech160nm_fixorc02aa1n02p5x5 g106(.a(new_n198), .b(new_n201), .out0(\s[18] ));
  norb02aa1n03x5               g107(.a(new_n184), .b(new_n201), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n200), .b(new_n199), .c(new_n195), .d(new_n196), .o1(new_n205));
  aoai13aa1n04x5               g110(.a(new_n205), .b(new_n204), .c(new_n187), .d(new_n193), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand42aa1n06x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\b[19] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(\a[20] ), .b(new_n211), .out0(new_n212));
  nand02aa1n03x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  aoi122aa1n06x5               g118(.a(new_n209), .b(new_n212), .c(new_n213), .d(new_n206), .e(new_n210), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n209), .o1(new_n215));
  nanb02aa1n12x5               g120(.a(new_n209), .b(new_n210), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nanp02aa1n03x5               g122(.a(new_n206), .b(new_n217), .o1(new_n218));
  nand42aa1n02x5               g123(.a(new_n212), .b(new_n213), .o1(new_n219));
  aoi012aa1n03x5               g124(.a(new_n219), .b(new_n218), .c(new_n215), .o1(new_n220));
  norp02aa1n03x5               g125(.a(new_n220), .b(new_n214), .o1(\s[20] ));
  nona23aa1n02x4               g126(.a(new_n217), .b(new_n184), .c(new_n201), .d(new_n219), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n209), .b(new_n213), .o1(new_n223));
  nor043aa1n03x5               g128(.a(new_n205), .b(new_n216), .c(new_n219), .o1(new_n224));
  nano22aa1n03x7               g129(.a(new_n224), .b(new_n212), .c(new_n223), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n187), .d(new_n193), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoi112aa1n02x5               g137(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n230), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n228), .c(new_n226), .d(new_n230), .o1(new_n234));
  norb02aa1n03x4               g139(.a(new_n234), .b(new_n233), .out0(\s[22] ));
  norp02aa1n02x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  nona23aa1n06x5               g141(.a(new_n213), .b(new_n210), .c(new_n209), .d(new_n236), .out0(new_n237));
  nor042aa1n06x5               g142(.a(new_n231), .b(new_n229), .o1(new_n238));
  nona23aa1n09x5               g143(.a(new_n238), .b(new_n184), .c(new_n237), .d(new_n201), .out0(new_n239));
  oai112aa1n03x5               g144(.a(new_n223), .b(new_n212), .c(new_n237), .d(new_n205), .o1(new_n240));
  inv040aa1n02x5               g145(.a(new_n228), .o1(new_n241));
  tech160nm_fioaoi03aa1n03p5x5 g146(.a(\a[22] ), .b(\b[21] ), .c(new_n241), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n240), .c(new_n238), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n239), .c(new_n187), .d(new_n193), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  xorc02aa1n12x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n246), .b(new_n248), .c(new_n244), .d(new_n247), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n247), .o1(new_n250));
  norb02aa1n02x7               g155(.a(new_n250), .b(new_n249), .out0(\s[24] ));
  nanp02aa1n02x5               g156(.a(new_n248), .b(new_n247), .o1(new_n252));
  nona23aa1n06x5               g157(.a(new_n238), .b(new_n203), .c(new_n252), .d(new_n237), .out0(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .out0(new_n254));
  norb02aa1n02x5               g159(.a(new_n248), .b(new_n254), .out0(new_n255));
  norp02aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n257));
  nanp03aa1n03x5               g162(.a(new_n242), .b(new_n247), .c(new_n248), .o1(new_n258));
  nona22aa1n09x5               g163(.a(new_n258), .b(new_n257), .c(new_n256), .out0(new_n259));
  aoi013aa1n02x4               g164(.a(new_n259), .b(new_n240), .c(new_n238), .d(new_n255), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n253), .c(new_n187), .d(new_n193), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  tech160nm_fixorc02aa1n03p5x5 g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  tech160nm_fixorc02aa1n05x5   g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n267));
  norb02aa1n03x4               g172(.a(new_n267), .b(new_n266), .out0(\s[26] ));
  nanp02aa1n02x5               g173(.a(new_n159), .b(new_n189), .o1(new_n269));
  norp03aa1n02x5               g174(.a(new_n171), .b(new_n188), .c(new_n179), .o1(new_n270));
  nona32aa1n02x4               g175(.a(new_n269), .b(new_n270), .c(new_n190), .d(new_n177), .out0(new_n271));
  and002aa1n06x5               g176(.a(new_n265), .b(new_n264), .o(new_n272));
  nano22aa1n03x7               g177(.a(new_n239), .b(new_n255), .c(new_n272), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n271), .c(new_n134), .d(new_n186), .o1(new_n274));
  nano22aa1n03x5               g179(.a(new_n225), .b(new_n238), .c(new_n255), .out0(new_n275));
  inv040aa1d32x5               g180(.a(\a[26] ), .o1(new_n276));
  inv000aa1d42x5               g181(.a(\b[25] ), .o1(new_n277));
  oaoi03aa1n02x5               g182(.a(new_n276), .b(new_n277), .c(new_n263), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  oaoi13aa1n09x5               g184(.a(new_n279), .b(new_n272), .c(new_n275), .d(new_n259), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n274), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  aobi12aa1n02x7               g189(.a(new_n281), .b(new_n280), .c(new_n274), .out0(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[27] ), .b(\a[28] ), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n285), .b(new_n284), .c(new_n286), .out0(new_n287));
  nona32aa1n02x5               g192(.a(new_n240), .b(new_n252), .c(new_n231), .d(new_n229), .out0(new_n288));
  inv000aa1n02x5               g193(.a(new_n259), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n272), .o1(new_n290));
  aoai13aa1n06x5               g195(.a(new_n278), .b(new_n290), .c(new_n288), .d(new_n289), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n281), .b(new_n291), .c(new_n197), .d(new_n273), .o1(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n286), .b(new_n292), .c(new_n284), .o1(new_n293));
  nor002aa1n02x5               g198(.a(new_n293), .b(new_n287), .o1(\s[28] ));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n281), .b(new_n286), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n291), .c(new_n197), .d(new_n273), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n295), .b(new_n297), .c(new_n298), .o1(new_n299));
  aobi12aa1n02x7               g204(.a(new_n296), .b(new_n280), .c(new_n274), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n295), .c(new_n298), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g208(.a(new_n281), .b(new_n295), .c(new_n286), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n291), .c(new_n197), .d(new_n273), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n03x5               g213(.a(new_n304), .b(new_n280), .c(new_n274), .out0(new_n309));
  nano22aa1n02x4               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  nor002aa1n02x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n304), .b(new_n307), .out0(new_n313));
  aobi12aa1n02x7               g218(.a(new_n313), .b(new_n280), .c(new_n274), .out0(new_n314));
  oao003aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n315));
  nano22aa1n02x4               g220(.a(new_n314), .b(new_n312), .c(new_n315), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n313), .b(new_n291), .c(new_n197), .d(new_n273), .o1(new_n317));
  tech160nm_fiaoi012aa1n05x5   g222(.a(new_n312), .b(new_n317), .c(new_n315), .o1(new_n318));
  nor002aa1n02x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .c(new_n131), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g230(.a(\a[6] ), .o1(new_n326));
  oaoi03aa1n02x5               g231(.a(new_n326), .b(new_n120), .c(new_n324), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g233(.a(\a[7] ), .b(\b[6] ), .c(new_n327), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n134), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


