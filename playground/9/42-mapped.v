// Benchmark "adder" written by ABC on Wed Jul 17 16:56:45 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n326, new_n327, new_n330, new_n332, new_n334, new_n335,
    new_n336, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n04x5   g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nanp02aa1n06x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n06x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n06x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  xorc02aa1n06x5               g011(.a(\a[3] ), .b(\b[2] ), .out0(new_n107));
  nanb03aa1n06x5               g012(.a(new_n103), .b(new_n107), .c(new_n106), .out0(new_n108));
  nor002aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  aoi012aa1n12x5               g014(.a(new_n104), .b(new_n109), .c(new_n105), .o1(new_n110));
  nanp02aa1n06x5               g015(.a(new_n108), .b(new_n110), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n08x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norb02aa1n02x7               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  nor042aa1n06x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n10x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  norb02aa1d21x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nand02aa1n04x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor002aa1n16x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(new_n119), .b(new_n118), .out0(new_n120));
  tech160nm_fixorc02aa1n05x5   g025(.a(\a[5] ), .b(\b[4] ), .out0(new_n121));
  nano32aa1n03x7               g026(.a(new_n120), .b(new_n121), .c(new_n114), .d(new_n117), .out0(new_n122));
  inv000aa1n06x5               g027(.a(new_n115), .o1(new_n123));
  nor042aa1n04x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  oai012aa1n06x5               g029(.a(new_n118), .b(new_n124), .c(new_n119), .o1(new_n125));
  nand02aa1d04x5               g030(.a(new_n125), .b(new_n123), .o1(new_n126));
  aoi022aa1d24x5               g031(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n127));
  aoi012aa1d18x5               g032(.a(new_n112), .b(new_n126), .c(new_n127), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n111), .d(new_n122), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n97), .b(new_n131), .c(new_n99), .out0(\s[10] ));
  and002aa1n02x7               g037(.a(\b[9] ), .b(\a[10] ), .o(new_n133));
  tech160nm_finand02aa1n03p5x5 g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor042aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  oa0022aa1n06x5               g041(.a(\b[9] ), .b(\a[10] ), .c(\b[8] ), .d(\a[9] ), .o(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n131), .d(new_n137), .o1(new_n138));
  nano23aa1n02x5               g043(.a(new_n112), .b(new_n115), .c(new_n116), .d(new_n113), .out0(new_n139));
  nanb03aa1n03x5               g044(.a(new_n120), .b(new_n139), .c(new_n121), .out0(new_n140));
  aoai13aa1n06x5               g045(.a(new_n128), .b(new_n140), .c(new_n108), .d(new_n110), .o1(new_n141));
  inv040aa1n02x5               g046(.a(new_n137), .o1(new_n142));
  norb03aa1n03x5               g047(.a(new_n134), .b(new_n133), .c(new_n135), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n141), .d(new_n130), .o1(new_n144));
  and002aa1n02x5               g049(.a(new_n138), .b(new_n144), .o(\s[11] ));
  inv000aa1d42x5               g050(.a(new_n135), .o1(new_n146));
  nor042aa1n03x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n03x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n144), .c(new_n146), .out0(\s[12] ));
  nano23aa1n03x7               g055(.a(new_n147), .b(new_n135), .c(new_n148), .d(new_n134), .out0(new_n151));
  and003aa1n02x5               g056(.a(new_n151), .b(new_n130), .c(new_n97), .o(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n129), .c(new_n111), .d(new_n122), .o1(new_n153));
  nanp03aa1d12x5               g058(.a(new_n143), .b(new_n142), .c(new_n149), .o1(new_n154));
  tech160nm_fiaoi012aa1n05x5   g059(.a(new_n147), .b(new_n135), .c(new_n148), .o1(new_n155));
  nand22aa1n06x5               g060(.a(new_n154), .b(new_n155), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(new_n153), .b(new_n157), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g064(.a(\a[14] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(\a[13] ), .o1(new_n161));
  inv000aa1d42x5               g066(.a(\b[12] ), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n158), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(new_n160), .out0(\s[14] ));
  nor002aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nor022aa1n16x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nano23aa1n06x5               g073(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n156), .c(new_n141), .d(new_n152), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n168), .b(new_n167), .c(new_n161), .d(new_n162), .o1(new_n171));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  nanp02aa1n02x5               g080(.a(new_n170), .b(new_n171), .o1(new_n176));
  nor002aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n172), .c(new_n176), .d(new_n173), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n176), .b(new_n174), .o1(new_n181));
  nona22aa1n02x4               g086(.a(new_n181), .b(new_n179), .c(new_n172), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n180), .o1(\s[16] ));
  nano23aa1n03x5               g088(.a(new_n172), .b(new_n177), .c(new_n178), .d(new_n173), .out0(new_n184));
  nanp02aa1n03x5               g089(.a(new_n184), .b(new_n169), .o1(new_n185));
  nano32aa1n03x7               g090(.a(new_n185), .b(new_n151), .c(new_n130), .d(new_n97), .out0(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n129), .c(new_n111), .d(new_n122), .o1(new_n187));
  nanb02aa1n03x5               g092(.a(new_n172), .b(new_n171), .out0(new_n188));
  aoi022aa1n02x5               g093(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n189));
  aoi012aa1n02x7               g094(.a(new_n177), .b(new_n188), .c(new_n189), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n185), .c(new_n154), .d(new_n155), .o1(new_n191));
  inv040aa1n08x5               g096(.a(new_n191), .o1(new_n192));
  nor042aa1n12x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nand42aa1d28x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n187), .c(new_n192), .out0(\s[17] ));
  inv000aa1d42x5               g101(.a(\a[18] ), .o1(new_n197));
  nand02aa1d08x5               g102(.a(new_n187), .b(new_n192), .o1(new_n198));
  tech160nm_fiaoi012aa1n05x5   g103(.a(new_n193), .b(new_n198), .c(new_n195), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(new_n197), .out0(\s[18] ));
  nor042aa1n09x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand42aa1d28x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nano23aa1d15x5               g107(.a(new_n193), .b(new_n201), .c(new_n202), .d(new_n194), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n203), .b(new_n191), .c(new_n141), .d(new_n186), .o1(new_n204));
  oa0012aa1n02x5               g109(.a(new_n202), .b(new_n201), .c(new_n193), .o(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  nor042aa1n06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1d27x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_finand02aa1n03p5x5 g116(.a(new_n204), .b(new_n206), .o1(new_n212));
  nor002aa1n06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1n08x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n18x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n207), .c(new_n212), .d(new_n208), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n209), .b(new_n205), .c(new_n198), .d(new_n203), .o1(new_n217));
  nona22aa1n02x5               g122(.a(new_n217), .b(new_n215), .c(new_n207), .out0(new_n218));
  nanp02aa1n03x5               g123(.a(new_n216), .b(new_n218), .o1(\s[20] ));
  nanb03aa1d24x5               g124(.a(new_n215), .b(new_n203), .c(new_n209), .out0(new_n220));
  nanb03aa1n09x5               g125(.a(new_n213), .b(new_n214), .c(new_n208), .out0(new_n221));
  orn002aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .o(new_n222));
  oai112aa1n06x5               g127(.a(new_n222), .b(new_n202), .c(new_n201), .d(new_n193), .o1(new_n223));
  aoi012aa1n09x5               g128(.a(new_n213), .b(new_n207), .c(new_n214), .o1(new_n224));
  oai012aa1d24x5               g129(.a(new_n224), .b(new_n223), .c(new_n221), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n04x5               g131(.a(new_n226), .b(new_n220), .c(new_n187), .d(new_n192), .o1(new_n227));
  nor042aa1n04x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nand42aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n220), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n225), .c(new_n198), .d(new_n231), .o1(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n227), .c(new_n230), .o1(\s[21] ));
  nor042aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  nanp02aa1n04x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n228), .c(new_n227), .d(new_n230), .o1(new_n237));
  nanp02aa1n04x5               g142(.a(new_n227), .b(new_n230), .o1(new_n238));
  nona22aa1n02x4               g143(.a(new_n238), .b(new_n236), .c(new_n228), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n239), .b(new_n237), .o1(\s[22] ));
  nano23aa1d12x5               g145(.a(new_n228), .b(new_n234), .c(new_n235), .d(new_n229), .out0(new_n241));
  nanb02aa1n02x5               g146(.a(new_n220), .b(new_n241), .out0(new_n242));
  aoi012aa1d18x5               g147(.a(new_n234), .b(new_n228), .c(new_n235), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n225), .c(new_n241), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n187), .d(new_n192), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n249), .o1(new_n251));
  nand02aa1n03x5               g156(.a(new_n246), .b(new_n249), .o1(new_n252));
  nona22aa1n02x4               g157(.a(new_n252), .b(new_n250), .c(new_n248), .out0(new_n253));
  nanp02aa1n02x5               g158(.a(new_n253), .b(new_n251), .o1(\s[24] ));
  norb02aa1n03x5               g159(.a(new_n249), .b(new_n250), .out0(new_n255));
  nano22aa1n03x7               g160(.a(new_n220), .b(new_n255), .c(new_n241), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n191), .c(new_n141), .d(new_n186), .o1(new_n257));
  nano22aa1n02x4               g162(.a(new_n213), .b(new_n208), .c(new_n214), .out0(new_n258));
  oai012aa1n02x5               g163(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .o1(new_n259));
  oab012aa1n02x4               g164(.a(new_n259), .b(new_n193), .c(new_n201), .out0(new_n260));
  inv040aa1n03x5               g165(.a(new_n224), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n241), .b(new_n261), .c(new_n260), .d(new_n258), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n255), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n264));
  oab012aa1n02x4               g169(.a(new_n264), .b(\a[24] ), .c(\b[23] ), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n263), .c(new_n262), .d(new_n243), .o1(new_n266));
  nanb02aa1n03x5               g171(.a(new_n266), .b(new_n257), .out0(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  tech160nm_fixnrc02aa1n05x5   g175(.a(\b[25] ), .b(\a[26] ), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n270), .b(new_n266), .c(new_n198), .d(new_n256), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .out0(new_n274));
  nanp02aa1n02x5               g179(.a(new_n272), .b(new_n274), .o1(\s[26] ));
  norb02aa1n12x5               g180(.a(new_n270), .b(new_n271), .out0(new_n276));
  inv030aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  nano23aa1d15x5               g182(.a(new_n277), .b(new_n220), .c(new_n255), .d(new_n241), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n191), .c(new_n141), .d(new_n186), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\a[26] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(\b[25] ), .o1(new_n281));
  oaoi03aa1n02x5               g186(.a(new_n280), .b(new_n281), .c(new_n269), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n282), .o1(new_n283));
  aoi012aa1n12x5               g188(.a(new_n283), .b(new_n266), .c(new_n276), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[27] ), .b(\b[26] ), .out0(new_n285));
  xnbna2aa1n03x5               g190(.a(new_n285), .b(new_n284), .c(new_n279), .out0(\s[27] ));
  nand42aa1n04x5               g191(.a(new_n284), .b(new_n279), .o1(new_n287));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  norp02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .o1(new_n289));
  nand42aa1n03x5               g194(.a(\b[27] ), .b(\a[28] ), .o1(new_n290));
  nanb02aa1n06x5               g195(.a(new_n289), .b(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n287), .d(new_n285), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n255), .b(new_n244), .c(new_n225), .d(new_n241), .o1(new_n293));
  aoai13aa1n06x5               g198(.a(new_n282), .b(new_n277), .c(new_n293), .d(new_n265), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n285), .b(new_n294), .c(new_n198), .d(new_n278), .o1(new_n295));
  nona22aa1n02x5               g200(.a(new_n295), .b(new_n291), .c(new_n288), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n292), .b(new_n296), .o1(\s[28] ));
  norb02aa1d27x5               g202(.a(new_n285), .b(new_n291), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n294), .c(new_n198), .d(new_n278), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n298), .o1(new_n300));
  oai012aa1n02x5               g205(.a(new_n290), .b(new_n289), .c(new_n288), .o1(new_n301));
  aoai13aa1n02x7               g206(.a(new_n301), .b(new_n300), .c(new_n284), .d(new_n279), .o1(new_n302));
  norp02aa1n02x5               g207(.a(\b[28] ), .b(\a[29] ), .o1(new_n303));
  nand42aa1n03x5               g208(.a(\b[28] ), .b(\a[29] ), .o1(new_n304));
  norb02aa1n02x5               g209(.a(new_n304), .b(new_n303), .out0(new_n305));
  oai022aa1n02x5               g210(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n306));
  aboi22aa1n03x5               g211(.a(new_n303), .b(new_n304), .c(new_n306), .d(new_n290), .out0(new_n307));
  aoi022aa1n03x5               g212(.a(new_n302), .b(new_n305), .c(new_n299), .d(new_n307), .o1(\s[29] ));
  xorb03aa1n02x5               g213(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g214(.a(new_n291), .b(new_n285), .c(new_n305), .out0(new_n310));
  aoai13aa1n06x5               g215(.a(new_n310), .b(new_n294), .c(new_n198), .d(new_n278), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n310), .o1(new_n312));
  aoi013aa1n02x4               g217(.a(new_n303), .b(new_n306), .c(new_n290), .d(new_n304), .o1(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n312), .c(new_n284), .d(new_n279), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .out0(new_n315));
  aoi113aa1n02x5               g220(.a(new_n315), .b(new_n303), .c(new_n306), .d(new_n304), .e(new_n290), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n311), .d(new_n316), .o1(\s[30] ));
  nanp03aa1n02x5               g222(.a(new_n298), .b(new_n305), .c(new_n315), .o1(new_n318));
  nanb02aa1n03x5               g223(.a(new_n318), .b(new_n287), .out0(new_n319));
  xorc02aa1n02x5               g224(.a(\a[31] ), .b(\b[30] ), .out0(new_n320));
  oao003aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n320), .out0(new_n322));
  aoai13aa1n04x5               g227(.a(new_n321), .b(new_n318), .c(new_n284), .d(new_n279), .o1(new_n323));
  aoi022aa1n03x5               g228(.a(new_n319), .b(new_n322), .c(new_n323), .d(new_n320), .o1(\s[31] ));
  xnrb03aa1n02x5               g229(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  obai22aa1n02x7               g230(.a(new_n105), .b(new_n104), .c(\a[3] ), .d(\b[2] ), .out0(new_n326));
  aoib12aa1n02x5               g231(.a(new_n326), .b(new_n107), .c(new_n103), .out0(new_n327));
  oaoi13aa1n02x5               g232(.a(new_n327), .b(new_n111), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g233(.a(new_n121), .b(new_n108), .c(new_n110), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g234(.a(new_n124), .b(new_n111), .c(new_n121), .o(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi012aa1n02x5               g236(.a(new_n119), .b(new_n330), .c(new_n118), .o1(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n332), .b(new_n123), .c(new_n116), .out0(\s[7] ));
  inv000aa1d42x5               g238(.a(new_n117), .o1(new_n334));
  aoi112aa1n03x4               g239(.a(new_n119), .b(new_n334), .c(new_n330), .d(new_n118), .o1(new_n335));
  nona22aa1n02x4               g240(.a(new_n116), .b(new_n335), .c(new_n114), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n114), .b(new_n335), .c(\b[6] ), .d(\a[7] ), .o1(new_n337));
  nanp02aa1n02x5               g242(.a(new_n336), .b(new_n337), .o1(\s[8] ));
  xorb03aa1n02x5               g243(.a(new_n141), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


