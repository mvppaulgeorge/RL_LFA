// Benchmark "adder" written by ABC on Thu Jul 18 09:17:52 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n312, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n334, new_n337, new_n339, new_n340, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n18x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand02aa1n04x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor002aa1n03x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  tech160nm_fioai012aa1n03p5x5 g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  norb02aa1n06x5               g008(.a(new_n100), .b(new_n101), .out0(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n102), .out0(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  aoi022aa1d18x5               g012(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n108));
  oai112aa1n06x5               g013(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n108), .o1(new_n109));
  nanp02aa1n03x5               g014(.a(new_n109), .b(new_n103), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norp02aa1n04x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanb02aa1n03x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  inv040aa1d32x5               g018(.a(\a[5] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[4] ), .o1(new_n115));
  nand02aa1d06x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand22aa1n03x5               g022(.a(new_n116), .b(new_n117), .o1(new_n118));
  norp02aa1n04x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nanp02aa1n04x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nor042aa1n04x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanp02aa1n04x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nona23aa1n03x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  nor043aa1n03x5               g028(.a(new_n123), .b(new_n118), .c(new_n113), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n111), .b(new_n112), .c(new_n115), .d(new_n114), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n120), .b(new_n121), .c(new_n119), .o1(new_n126));
  oai012aa1n02x5               g031(.a(new_n126), .b(new_n123), .c(new_n125), .o1(new_n127));
  xorc02aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n110), .d(new_n124), .o1(new_n129));
  obai22aa1n02x7               g034(.a(new_n129), .b(new_n99), .c(new_n97), .d(new_n98), .out0(new_n130));
  nona32aa1n02x4               g035(.a(new_n129), .b(new_n99), .c(new_n98), .d(new_n97), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n130), .b(new_n131), .o1(\s[10] ));
  inv000aa1d42x5               g037(.a(new_n98), .o1(new_n133));
  nor022aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n131), .c(new_n133), .out0(\s[11] ));
  inv000aa1n03x5               g042(.a(new_n134), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n131), .b(new_n136), .c(new_n98), .out0(new_n139));
  nor002aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n03x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  xobna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n138), .out0(\s[12] ));
  nor002aa1n02x5               g048(.a(new_n98), .b(new_n97), .o1(new_n144));
  nano23aa1n06x5               g049(.a(new_n134), .b(new_n140), .c(new_n141), .d(new_n135), .out0(new_n145));
  and003aa1n02x5               g050(.a(new_n145), .b(new_n128), .c(new_n144), .o(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n127), .c(new_n110), .d(new_n124), .o1(new_n147));
  oai022aa1n12x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  oaoi03aa1n02x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n149));
  aoi013aa1n02x4               g054(.a(new_n149), .b(new_n145), .c(new_n148), .d(new_n133), .o1(new_n150));
  xnrc02aa1n12x5               g055(.a(\b[12] ), .b(\a[13] ), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n150), .out0(\s[13] ));
  orn002aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .o(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n147), .d(new_n150), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n03x5               g060(.a(new_n147), .b(new_n150), .o1(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  nona22aa1n03x5               g062(.a(new_n156), .b(new_n151), .c(new_n157), .out0(new_n158));
  oaih22aa1d12x5               g063(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n159));
  aob012aa1n12x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(new_n160));
  nor002aa1n10x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nand42aa1n08x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n158), .c(new_n160), .out0(\s[15] ));
  inv000aa1d42x5               g069(.a(new_n161), .o1(new_n165));
  aoi112aa1n03x5               g070(.a(new_n157), .b(new_n151), .c(new_n147), .d(new_n150), .o1(new_n166));
  inv040aa1d30x5               g071(.a(new_n160), .o1(new_n167));
  oai012aa1n02x5               g072(.a(new_n163), .b(new_n166), .c(new_n167), .o1(new_n168));
  nor002aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand42aa1n03x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aobi12aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n165), .out0(new_n172));
  nand42aa1n03x5               g077(.a(new_n158), .b(new_n160), .o1(new_n173));
  aoi112aa1n03x4               g078(.a(new_n161), .b(new_n171), .c(new_n173), .d(new_n162), .o1(new_n174));
  norp02aa1n02x5               g079(.a(new_n172), .b(new_n174), .o1(\s[16] ));
  inv030aa1d32x5               g080(.a(\a[17] ), .o1(new_n176));
  nano23aa1n09x5               g081(.a(new_n119), .b(new_n121), .c(new_n122), .d(new_n120), .out0(new_n177));
  nona22aa1n02x4               g082(.a(new_n177), .b(new_n113), .c(new_n118), .out0(new_n178));
  oaoi03aa1n02x5               g083(.a(\a[6] ), .b(\b[5] ), .c(new_n116), .o1(new_n179));
  oai022aa1n02x5               g084(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n180));
  aoi022aa1n06x5               g085(.a(new_n177), .b(new_n179), .c(new_n120), .d(new_n180), .o1(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n178), .c(new_n103), .d(new_n109), .o1(new_n182));
  nano23aa1n03x5               g087(.a(new_n161), .b(new_n169), .c(new_n170), .d(new_n162), .out0(new_n183));
  nona22aa1n03x5               g088(.a(new_n183), .b(new_n157), .c(new_n151), .out0(new_n184));
  nano32aa1n03x7               g089(.a(new_n184), .b(new_n145), .c(new_n128), .d(new_n144), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n148), .o1(new_n186));
  nona22aa1n02x5               g091(.a(new_n145), .b(new_n186), .c(new_n98), .out0(new_n187));
  inv020aa1n02x5               g092(.a(new_n149), .o1(new_n188));
  oaoi03aa1n02x5               g093(.a(\a[16] ), .b(\b[15] ), .c(new_n165), .o1(new_n189));
  aoi012aa1n06x5               g094(.a(new_n189), .b(new_n167), .c(new_n183), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n184), .c(new_n187), .d(new_n188), .o1(new_n191));
  aoi012aa1n06x5               g096(.a(new_n191), .b(new_n182), .c(new_n185), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(new_n176), .out0(\s[17] ));
  inv030aa1d32x5               g098(.a(\b[16] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(new_n194), .b(new_n176), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n191), .c(new_n182), .d(new_n185), .o1(new_n197));
  nor002aa1d32x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nanp02aa1n24x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  norb02aa1n12x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n195), .out0(\s[18] ));
  inv000aa1d42x5               g106(.a(new_n200), .o1(new_n202));
  aoai13aa1n12x5               g107(.a(new_n199), .b(new_n198), .c(new_n176), .d(new_n194), .o1(new_n203));
  nor042aa1n06x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand42aa1n06x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  oaoi13aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(new_n197), .d(new_n202), .o1(new_n207));
  nano22aa1n03x7               g112(.a(new_n192), .b(new_n196), .c(new_n200), .out0(new_n208));
  nano22aa1n02x4               g113(.a(new_n208), .b(new_n203), .c(new_n206), .out0(new_n209));
  norp02aa1n02x5               g114(.a(new_n207), .b(new_n209), .o1(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n03x5               g116(.a(new_n204), .o1(new_n212));
  inv040aa1n06x5               g117(.a(new_n203), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n206), .o1(new_n214));
  oai012aa1n03x5               g119(.a(new_n214), .b(new_n208), .c(new_n213), .o1(new_n215));
  nor022aa1n06x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand42aa1n04x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanb02aa1n02x5               g122(.a(new_n216), .b(new_n217), .out0(new_n218));
  aoi012aa1n03x5               g123(.a(new_n218), .b(new_n215), .c(new_n212), .o1(new_n219));
  nano22aa1n03x5               g124(.a(new_n207), .b(new_n212), .c(new_n218), .out0(new_n220));
  nor002aa1n02x5               g125(.a(new_n219), .b(new_n220), .o1(\s[20] ));
  nano23aa1n03x7               g126(.a(new_n204), .b(new_n216), .c(new_n217), .d(new_n205), .out0(new_n222));
  nanp03aa1n02x5               g127(.a(new_n222), .b(new_n196), .c(new_n200), .o1(new_n223));
  inv000aa1n02x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n191), .c(new_n182), .d(new_n185), .o1(new_n225));
  nona23aa1n09x5               g130(.a(new_n217), .b(new_n205), .c(new_n204), .d(new_n216), .out0(new_n226));
  oaoi03aa1n06x5               g131(.a(\a[20] ), .b(\b[19] ), .c(new_n212), .o1(new_n227));
  inv040aa1n03x5               g132(.a(new_n227), .o1(new_n228));
  oai012aa1d24x5               g133(.a(new_n228), .b(new_n226), .c(new_n203), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[20] ), .b(\a[21] ), .out0(new_n231));
  xobna2aa1n03x5               g136(.a(new_n231), .b(new_n225), .c(new_n230), .out0(\s[21] ));
  orn002aa1n03x5               g137(.a(\a[21] ), .b(\b[20] ), .o(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n225), .d(new_n230), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[21] ), .c(\a[22] ), .out0(\s[22] ));
  xorc02aa1n12x5               g140(.a(\a[22] ), .b(\b[21] ), .out0(new_n236));
  norb02aa1n15x5               g141(.a(new_n236), .b(new_n231), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  oaoi03aa1n12x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n233), .o1(new_n239));
  aoi012aa1d18x5               g144(.a(new_n239), .b(new_n229), .c(new_n237), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  oaoi13aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n225), .d(new_n238), .o1(new_n242));
  nano22aa1n03x7               g147(.a(new_n192), .b(new_n224), .c(new_n237), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n243), .b(new_n240), .c(new_n241), .out0(new_n244));
  norp02aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(\s[23] ));
  orn002aa1n24x5               g150(.a(\a[23] ), .b(\b[22] ), .o(new_n246));
  inv000aa1d42x5               g151(.a(new_n240), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n241), .o1(new_n248));
  tech160nm_fioai012aa1n05x5   g153(.a(new_n248), .b(new_n243), .c(new_n247), .o1(new_n249));
  tech160nm_fixnrc02aa1n04x5   g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  aoi012aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n246), .o1(new_n251));
  nano22aa1n03x5               g156(.a(new_n242), .b(new_n246), .c(new_n250), .out0(new_n252));
  norp02aa1n03x5               g157(.a(new_n251), .b(new_n252), .o1(\s[24] ));
  aoai13aa1n06x5               g158(.a(new_n237), .b(new_n227), .c(new_n222), .d(new_n213), .o1(new_n254));
  inv000aa1n06x5               g159(.a(new_n239), .o1(new_n255));
  norp02aa1n04x5               g160(.a(new_n250), .b(new_n241), .o1(new_n256));
  inv020aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fioaoi03aa1n02p5x5 g162(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .o1(new_n258));
  inv030aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n257), .c(new_n254), .d(new_n255), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  nanp02aa1n03x5               g166(.a(new_n182), .b(new_n185), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n187), .b(new_n188), .o1(new_n263));
  nona23aa1n02x4               g168(.a(new_n170), .b(new_n162), .c(new_n161), .d(new_n169), .out0(new_n264));
  norp03aa1n02x5               g169(.a(new_n264), .b(new_n157), .c(new_n151), .o1(new_n265));
  aobi12aa1n02x7               g170(.a(new_n190), .b(new_n263), .c(new_n265), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n262), .b(new_n266), .o1(new_n267));
  nona32aa1n03x5               g172(.a(new_n267), .b(new_n257), .c(new_n238), .d(new_n223), .out0(new_n268));
  xnrc02aa1n12x5               g173(.a(\b[24] ), .b(\a[25] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n261), .out0(\s[25] ));
  nor042aa1n03x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  nano23aa1n02x4               g178(.a(new_n231), .b(new_n250), .c(new_n248), .d(new_n236), .out0(new_n274));
  nano22aa1n03x7               g179(.a(new_n192), .b(new_n224), .c(new_n274), .out0(new_n275));
  tech160nm_fioai012aa1n05x5   g180(.a(new_n270), .b(new_n275), .c(new_n260), .o1(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[25] ), .b(\a[26] ), .out0(new_n277));
  aoi012aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n273), .o1(new_n278));
  tech160nm_fiaoi012aa1n03p5x5 g183(.a(new_n269), .b(new_n268), .c(new_n261), .o1(new_n279));
  nano22aa1n03x7               g184(.a(new_n279), .b(new_n273), .c(new_n277), .out0(new_n280));
  norp02aa1n03x5               g185(.a(new_n278), .b(new_n280), .o1(\s[26] ));
  xorc02aa1n12x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  nor042aa1n04x5               g188(.a(new_n277), .b(new_n269), .o1(new_n284));
  nano32aa1n03x7               g189(.a(new_n223), .b(new_n284), .c(new_n237), .d(new_n256), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n191), .c(new_n182), .d(new_n185), .o1(new_n286));
  nanp02aa1n09x5               g191(.a(new_n260), .b(new_n284), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .carry(new_n288));
  aoi013aa1n06x4               g193(.a(new_n283), .b(new_n287), .c(new_n288), .d(new_n286), .o1(new_n289));
  aoai13aa1n02x7               g194(.a(new_n256), .b(new_n239), .c(new_n229), .d(new_n237), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n284), .o1(new_n291));
  aoai13aa1n04x5               g196(.a(new_n288), .b(new_n291), .c(new_n290), .d(new_n259), .o1(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n286), .c(new_n283), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n289), .b(new_n293), .o1(\s[27] ));
  nor042aa1n03x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n285), .b(new_n262), .c(new_n266), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n282), .b(new_n292), .c(new_n297), .o1(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[27] ), .b(\a[28] ), .out0(new_n299));
  aoi012aa1n02x7               g204(.a(new_n299), .b(new_n298), .c(new_n296), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n289), .b(new_n296), .c(new_n299), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n300), .b(new_n301), .o1(\s[28] ));
  xnrc02aa1n02x5               g207(.a(\b[28] ), .b(\a[29] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n282), .b(new_n299), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n292), .c(new_n297), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n303), .b(new_n305), .c(new_n306), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n304), .o1(new_n308));
  aoi013aa1n03x5               g213(.a(new_n308), .b(new_n287), .c(new_n288), .d(new_n286), .o1(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n303), .c(new_n306), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n307), .b(new_n310), .o1(\s[29] ));
  nanp02aa1n02x5               g216(.a(\b[0] ), .b(\a[1] ), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g218(.a(\b[29] ), .b(\a[30] ), .out0(new_n314));
  norb03aa1n03x5               g219(.a(new_n282), .b(new_n303), .c(new_n299), .out0(new_n315));
  oaih12aa1n02x5               g220(.a(new_n315), .b(new_n292), .c(new_n297), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .carry(new_n317));
  aoi012aa1n02x5               g222(.a(new_n314), .b(new_n316), .c(new_n317), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n315), .o1(new_n319));
  aoi013aa1n03x5               g224(.a(new_n319), .b(new_n287), .c(new_n288), .d(new_n286), .o1(new_n320));
  nano22aa1n03x5               g225(.a(new_n320), .b(new_n314), .c(new_n317), .out0(new_n321));
  norp02aa1n03x5               g226(.a(new_n318), .b(new_n321), .o1(\s[30] ));
  norb02aa1n06x5               g227(.a(new_n315), .b(new_n314), .out0(new_n323));
  inv000aa1n02x5               g228(.a(new_n323), .o1(new_n324));
  aoi013aa1n03x5               g229(.a(new_n324), .b(new_n287), .c(new_n288), .d(new_n286), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n326));
  xnrc02aa1n02x5               g231(.a(\b[30] ), .b(\a[31] ), .out0(new_n327));
  nano22aa1n03x5               g232(.a(new_n325), .b(new_n326), .c(new_n327), .out0(new_n328));
  oaih12aa1n02x5               g233(.a(new_n323), .b(new_n292), .c(new_n297), .o1(new_n329));
  tech160nm_fiaoi012aa1n02p5x5 g234(.a(new_n327), .b(new_n329), .c(new_n326), .o1(new_n330));
  norp02aa1n03x5               g235(.a(new_n330), .b(new_n328), .o1(\s[31] ));
  norp02aa1n02x5               g236(.a(new_n108), .b(new_n107), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi13aa1n02x5               g238(.a(new_n102), .b(new_n105), .c(new_n108), .d(new_n107), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(new_n334), .b(new_n104), .out0(\s[4] ));
  xobna2aa1n03x5               g240(.a(new_n118), .b(new_n109), .c(new_n103), .out0(\s[5] ));
  aoai13aa1n02x5               g241(.a(new_n116), .b(new_n118), .c(new_n109), .d(new_n103), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi112aa1n02x5               g243(.a(new_n113), .b(new_n118), .c(new_n109), .d(new_n103), .o1(new_n339));
  norp02aa1n02x5               g244(.a(new_n339), .b(new_n179), .o1(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi13aa1n02x5               g246(.a(new_n121), .b(new_n122), .c(new_n339), .d(new_n179), .o1(new_n342));
  xnrb03aa1n02x5               g247(.a(new_n342), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n182), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


