// Benchmark "adder" written by ABC on Wed Jul 17 18:37:52 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n334, new_n337, new_n339, new_n340, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1n03x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n09x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nand02aa1n04x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nor022aa1n04x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand42aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nona23aa1d18x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  nor042aa1n04x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[2] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[1] ), .o1(new_n109));
  nand42aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  tech160nm_fioaoi03aa1n05x5   g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  norp02aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  tech160nm_fiao0012aa1n02p5x5 g021(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n117));
  oabi12aa1n06x5               g022(.a(new_n117), .b(new_n116), .c(new_n111), .out0(new_n118));
  inv000aa1d42x5               g023(.a(new_n97), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n99), .b(new_n98), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[5] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[4] ), .o1(new_n122));
  aoai13aa1n04x5               g027(.a(new_n103), .b(new_n102), .c(new_n121), .d(new_n122), .o1(new_n123));
  oai112aa1n02x7               g028(.a(new_n119), .b(new_n120), .c(new_n101), .d(new_n123), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n124), .b(new_n118), .c(new_n107), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1d18x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand42aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  nor002aa1d32x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nor022aa1n16x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand02aa1n08x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nano23aa1n03x7               g039(.a(new_n131), .b(new_n133), .c(new_n134), .d(new_n132), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n124), .c(new_n118), .d(new_n107), .o1(new_n136));
  oai012aa1d24x5               g041(.a(new_n134), .b(new_n133), .c(new_n131), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n130), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n128), .o1(new_n139));
  inv000aa1n03x5               g044(.a(new_n125), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n137), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n130), .b(new_n141), .c(new_n140), .d(new_n135), .o1(new_n142));
  nor042aa1d18x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n139), .out0(\s[12] ));
  nor022aa1n16x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n143), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n128), .b(new_n144), .o1(new_n151));
  nona23aa1n09x5               g056(.a(new_n144), .b(new_n129), .c(new_n128), .d(new_n143), .out0(new_n152));
  oai112aa1n06x5               g057(.a(new_n151), .b(new_n150), .c(new_n152), .d(new_n137), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  nona23aa1n02x4               g059(.a(new_n134), .b(new_n132), .c(new_n131), .d(new_n133), .out0(new_n155));
  norp02aa1n02x5               g060(.a(new_n152), .b(new_n155), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n124), .c(new_n118), .d(new_n107), .o1(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n149), .b(new_n157), .c(new_n154), .out0(\s[13] ));
  inv000aa1d42x5               g063(.a(new_n147), .o1(new_n159));
  aob012aa1n02x5               g064(.a(new_n149), .b(new_n157), .c(new_n154), .out0(new_n160));
  nor002aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n160), .c(new_n159), .out0(\s[14] ));
  nona23aa1n08x5               g069(.a(new_n162), .b(new_n148), .c(new_n147), .d(new_n161), .out0(new_n165));
  oai012aa1n02x5               g070(.a(new_n162), .b(new_n161), .c(new_n147), .o1(new_n166));
  aoai13aa1n04x5               g071(.a(new_n166), .b(new_n165), .c(new_n157), .d(new_n154), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1n03x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  aoi112aa1n02x5               g079(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n171), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n174), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(\s[16] ));
  nano23aa1n02x5               g082(.a(new_n169), .b(new_n172), .c(new_n173), .d(new_n170), .out0(new_n178));
  nano23aa1n03x5               g083(.a(new_n165), .b(new_n152), .c(new_n178), .d(new_n135), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n124), .c(new_n107), .d(new_n118), .o1(new_n180));
  nano22aa1n03x7               g085(.a(new_n165), .b(new_n171), .c(new_n174), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  obai22aa1n02x7               g087(.a(new_n178), .b(new_n166), .c(\a[16] ), .d(\b[15] ), .out0(new_n183));
  aoi112aa1n09x5               g088(.a(new_n183), .b(new_n182), .c(new_n153), .d(new_n181), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n180), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[16] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  oao003aa1n02x5               g094(.a(new_n108), .b(new_n109), .c(new_n110), .carry(new_n190));
  nano23aa1n02x4               g095(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n191));
  aoai13aa1n02x5               g096(.a(new_n107), .b(new_n117), .c(new_n190), .d(new_n191), .o1(new_n192));
  norp02aa1n02x5               g097(.a(new_n101), .b(new_n123), .o1(new_n193));
  nano22aa1n02x4               g098(.a(new_n193), .b(new_n119), .c(new_n120), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(new_n181), .b(new_n156), .o1(new_n195));
  tech160nm_fiaoi012aa1n03p5x5 g100(.a(new_n195), .b(new_n192), .c(new_n194), .o1(new_n196));
  nand02aa1n03x5               g101(.a(new_n153), .b(new_n181), .o1(new_n197));
  nano22aa1n02x4               g102(.a(new_n166), .b(new_n171), .c(new_n174), .out0(new_n198));
  nona32aa1n06x5               g103(.a(new_n197), .b(new_n198), .c(new_n182), .d(new_n172), .out0(new_n199));
  oai012aa1n02x5               g104(.a(new_n185), .b(new_n199), .c(new_n196), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand42aa1n03x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nanb02aa1n09x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  xobna2aa1n03x5               g108(.a(new_n203), .b(new_n200), .c(new_n189), .out0(\s[18] ));
  nanp02aa1n02x5               g109(.a(\b[16] ), .b(\a[17] ), .o1(new_n205));
  nano22aa1n12x5               g110(.a(new_n203), .b(new_n189), .c(new_n205), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n202), .b(new_n201), .c(new_n187), .d(new_n188), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n207), .c(new_n184), .d(new_n180), .o1(new_n209));
  xorb03aa1n03x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n16x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1d04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nanb02aa1d24x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\a[20] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[19] ), .o1(new_n217));
  nand42aa1n02x5               g122(.a(new_n217), .b(new_n216), .o1(new_n218));
  nand42aa1n02x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1d08x5               g124(.a(new_n218), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoi112aa1n06x5               g126(.a(new_n212), .b(new_n221), .c(new_n209), .d(new_n215), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n212), .o1(new_n223));
  nanp02aa1n06x5               g128(.a(new_n209), .b(new_n215), .o1(new_n224));
  tech160nm_fiaoi012aa1n05x5   g129(.a(new_n220), .b(new_n224), .c(new_n223), .o1(new_n225));
  norp02aa1n03x5               g130(.a(new_n225), .b(new_n222), .o1(\s[20] ));
  nona22aa1n02x4               g131(.a(new_n206), .b(new_n214), .c(new_n220), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n212), .b(new_n219), .o1(new_n228));
  nor003aa1n03x5               g133(.a(new_n208), .b(new_n214), .c(new_n220), .o1(new_n229));
  nano22aa1n03x7               g134(.a(new_n229), .b(new_n218), .c(new_n228), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n227), .c(new_n184), .d(new_n180), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  tech160nm_fixorc02aa1n03p5x5 g138(.a(\a[21] ), .b(\b[20] ), .out0(new_n234));
  tech160nm_fixorc02aa1n05x5   g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  aoi112aa1n02x7               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n233), .o1(new_n237));
  nand42aa1n04x5               g142(.a(new_n231), .b(new_n234), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n235), .o1(new_n239));
  aoi012aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .o1(new_n240));
  norp02aa1n03x5               g145(.a(new_n240), .b(new_n236), .o1(\s[22] ));
  norp02aa1n02x5               g146(.a(\b[19] ), .b(\a[20] ), .o1(new_n242));
  nona23aa1n06x5               g147(.a(new_n219), .b(new_n213), .c(new_n212), .d(new_n242), .out0(new_n243));
  oai112aa1n02x5               g148(.a(new_n228), .b(new_n218), .c(new_n243), .d(new_n208), .o1(new_n244));
  and002aa1n02x5               g149(.a(new_n235), .b(new_n234), .o(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n244), .c(new_n245), .o1(new_n247));
  nona23aa1n02x4               g152(.a(new_n234), .b(new_n206), .c(new_n239), .d(new_n243), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n247), .b(new_n248), .c(new_n184), .d(new_n180), .o1(new_n249));
  xorb03aa1n03x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  nor002aa1n04x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  aoi112aa1n03x5               g161(.a(new_n251), .b(new_n256), .c(new_n249), .d(new_n253), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n251), .o1(new_n258));
  nanp02aa1n06x5               g163(.a(new_n249), .b(new_n253), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n256), .o1(new_n260));
  aoi012aa1n03x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .o1(new_n261));
  norp02aa1n03x5               g166(.a(new_n261), .b(new_n257), .o1(\s[24] ));
  nano23aa1d15x5               g167(.a(new_n251), .b(new_n254), .c(new_n255), .d(new_n252), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  nona23aa1n02x4               g169(.a(new_n245), .b(new_n206), .c(new_n264), .d(new_n243), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n266));
  aoi112aa1n06x5               g171(.a(new_n266), .b(new_n254), .c(new_n263), .d(new_n246), .o1(new_n267));
  nand23aa1d12x5               g172(.a(new_n263), .b(new_n234), .c(new_n235), .o1(new_n268));
  oai012aa1n18x5               g173(.a(new_n267), .b(new_n230), .c(new_n268), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n265), .c(new_n184), .d(new_n180), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  tech160nm_fixorc02aa1n03p5x5 g179(.a(\a[26] ), .b(\b[25] ), .out0(new_n275));
  aoi112aa1n03x4               g180(.a(new_n273), .b(new_n275), .c(new_n271), .d(new_n274), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n273), .o1(new_n277));
  nand42aa1n04x5               g182(.a(new_n271), .b(new_n274), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n275), .o1(new_n279));
  aoi012aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n277), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n276), .o1(\s[26] ));
  and002aa1n02x5               g186(.a(new_n275), .b(new_n274), .o(new_n282));
  nano22aa1n03x7               g187(.a(new_n248), .b(new_n263), .c(new_n282), .out0(new_n283));
  oai012aa1n06x5               g188(.a(new_n283), .b(new_n199), .c(new_n196), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .o1(new_n285));
  aoi112aa1n02x5               g190(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n286));
  aoi112aa1n06x5               g191(.a(new_n285), .b(new_n286), .c(new_n269), .d(new_n282), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[27] ), .b(\b[26] ), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n284), .c(new_n287), .out0(\s[27] ));
  norp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  inv040aa1n03x5               g195(.a(new_n290), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n288), .o1(new_n292));
  tech160nm_fiaoi012aa1n05x5   g197(.a(new_n292), .b(new_n284), .c(new_n287), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n293), .b(new_n291), .c(new_n294), .out0(new_n295));
  nand02aa1d06x5               g200(.a(new_n184), .b(new_n180), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n263), .b(new_n246), .o1(new_n297));
  nona22aa1n02x4               g202(.a(new_n297), .b(new_n266), .c(new_n254), .out0(new_n298));
  inv040aa1n03x5               g203(.a(new_n268), .o1(new_n299));
  aoai13aa1n02x5               g204(.a(new_n282), .b(new_n298), .c(new_n244), .d(new_n299), .o1(new_n300));
  nona22aa1n02x4               g205(.a(new_n300), .b(new_n286), .c(new_n285), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n288), .b(new_n301), .c(new_n296), .d(new_n283), .o1(new_n302));
  aoi012aa1n03x5               g207(.a(new_n294), .b(new_n302), .c(new_n291), .o1(new_n303));
  nor002aa1n02x5               g208(.a(new_n303), .b(new_n295), .o1(\s[28] ));
  norb02aa1n02x5               g209(.a(new_n288), .b(new_n294), .out0(new_n305));
  aoai13aa1n06x5               g210(.a(new_n305), .b(new_n301), .c(new_n296), .d(new_n283), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[28] ), .b(\a[29] ), .out0(new_n308));
  tech160nm_fiaoi012aa1n05x5   g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n305), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n310), .b(new_n284), .c(new_n287), .o1(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n308), .out0(new_n312));
  norp02aa1n03x5               g217(.a(new_n309), .b(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g219(.a(new_n288), .b(new_n308), .c(new_n294), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n301), .c(new_n296), .d(new_n283), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  aoi012aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n315), .o1(new_n320));
  tech160nm_fiaoi012aa1n02p5x5 g225(.a(new_n320), .b(new_n284), .c(new_n287), .o1(new_n321));
  nano22aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n318), .out0(new_n322));
  norp02aa1n03x5               g227(.a(new_n319), .b(new_n322), .o1(\s[30] ));
  norb02aa1n09x5               g228(.a(new_n315), .b(new_n318), .out0(new_n324));
  inv000aa1d42x5               g229(.a(new_n324), .o1(new_n325));
  tech160nm_fiaoi012aa1n04x5   g230(.a(new_n325), .b(new_n284), .c(new_n287), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  nano22aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n328), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n324), .b(new_n301), .c(new_n296), .d(new_n283), .o1(new_n330));
  tech160nm_fiaoi012aa1n02p5x5 g235(.a(new_n328), .b(new_n330), .c(new_n327), .o1(new_n331));
  nor002aa1n02x5               g236(.a(new_n331), .b(new_n329), .o1(\s[31] ));
  xnrb03aa1n02x5               g237(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g238(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g240(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g241(.a(new_n121), .b(new_n122), .c(new_n118), .o1(new_n337));
  xnrb03aa1n02x5               g242(.a(new_n337), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g243(.a(new_n106), .o1(new_n339));
  aobi12aa1n02x5               g244(.a(new_n123), .b(new_n118), .c(new_n339), .out0(new_n340));
  xnrb03aa1n02x5               g245(.a(new_n340), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g246(.a(\a[7] ), .b(\b[6] ), .c(new_n340), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g248(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


