// Benchmark "adder" written by ABC on Wed Jul 17 17:25:49 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n327,
    new_n328, new_n330, new_n331, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nand02aa1n03x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  ao0012aa1n03x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o(new_n101));
  inv000aa1d42x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  tech160nm_fioaoi03aa1n04x5   g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n107));
  oabi12aa1n06x5               g012(.a(new_n101), .b(new_n107), .c(new_n105), .out0(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor042aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d24x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n03x5               g017(.a(new_n111), .b(new_n109), .c(new_n112), .d(new_n110), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[5] ), .b(\b[4] ), .out0(new_n114));
  nor042aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand22aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norb02aa1n06x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nano22aa1n03x7               g022(.a(new_n113), .b(new_n114), .c(new_n117), .out0(new_n118));
  nor022aa1n04x5               g023(.a(new_n112), .b(new_n110), .o1(new_n119));
  nor002aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  aoai13aa1n04x5               g025(.a(new_n111), .b(new_n115), .c(new_n120), .d(new_n116), .o1(new_n121));
  aoi022aa1n12x5               g026(.a(new_n121), .b(new_n119), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  norp02aa1n09x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  nand42aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x7               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n122), .c(new_n108), .d(new_n118), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n03x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n97), .out0(\s[10] ));
  aobi12aa1n02x5               g035(.a(new_n129), .b(new_n126), .c(new_n97), .out0(new_n131));
  oai012aa1d24x5               g036(.a(new_n128), .b(new_n127), .c(new_n123), .o1(new_n132));
  inv040aa1n03x5               g037(.a(new_n132), .o1(new_n133));
  nanp02aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor022aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  oabi12aa1n02x5               g041(.a(new_n136), .b(new_n131), .c(new_n133), .out0(new_n137));
  nano22aa1n03x7               g042(.a(new_n131), .b(new_n132), .c(new_n136), .out0(new_n138));
  nanb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[11] ));
  nor022aa1n08x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(\a[11] ), .d(\b[10] ), .o1(new_n143));
  nona22aa1n09x5               g048(.a(new_n134), .b(new_n138), .c(new_n142), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n143), .o1(\s[12] ));
  nona23aa1d18x5               g050(.a(new_n134), .b(new_n141), .c(new_n140), .d(new_n135), .out0(new_n146));
  nano22aa1n03x7               g051(.a(new_n146), .b(new_n125), .c(new_n129), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n122), .c(new_n108), .d(new_n118), .o1(new_n148));
  oa0012aa1n06x5               g053(.a(new_n141), .b(new_n140), .c(new_n135), .o(new_n149));
  oabi12aa1n18x5               g054(.a(new_n149), .b(new_n132), .c(new_n146), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nand42aa1n03x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nor002aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  nand22aa1n03x5               g060(.a(new_n148), .b(new_n151), .o1(new_n156));
  nor042aa1n04x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n153), .c(new_n156), .d(new_n152), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n153), .c(new_n156), .d(new_n152), .o1(new_n161));
  nanb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(\s[14] ));
  nano23aa1d12x5               g067(.a(new_n157), .b(new_n153), .c(new_n158), .d(new_n152), .out0(new_n163));
  oaih12aa1n06x5               g068(.a(new_n158), .b(new_n157), .c(new_n153), .o1(new_n164));
  aob012aa1n02x5               g069(.a(new_n164), .b(new_n156), .c(new_n163), .out0(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanp02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor002aa1d32x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n06x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n167), .c(new_n165), .d(new_n170), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n164), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n170), .b(new_n175), .c(new_n156), .d(new_n163), .o1(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n173), .c(new_n167), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n174), .b(new_n177), .o1(\s[16] ));
  aoi012aa1n06x5               g083(.a(new_n122), .b(new_n108), .c(new_n118), .o1(new_n179));
  nona23aa1d24x5               g084(.a(new_n168), .b(new_n172), .c(new_n171), .d(new_n167), .out0(new_n180));
  nona32aa1n09x5               g085(.a(new_n147), .b(new_n180), .c(new_n159), .d(new_n154), .out0(new_n181));
  oai012aa1n02x5               g086(.a(new_n172), .b(new_n171), .c(new_n167), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n180), .o1(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n175), .c(new_n150), .d(new_n163), .o1(new_n184));
  oai112aa1n06x5               g089(.a(new_n182), .b(new_n184), .c(new_n179), .d(new_n181), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor022aa1n06x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  oao003aa1n02x5               g093(.a(new_n102), .b(new_n103), .c(new_n104), .carry(new_n189));
  nano23aa1n03x5               g094(.a(new_n98), .b(new_n100), .c(new_n106), .d(new_n99), .out0(new_n190));
  aoi012aa1n06x5               g095(.a(new_n101), .b(new_n190), .c(new_n189), .o1(new_n191));
  nanb02aa1n03x5               g096(.a(new_n110), .b(new_n109), .out0(new_n192));
  nanb02aa1n12x5               g097(.a(new_n112), .b(new_n111), .out0(new_n193));
  nona23aa1n09x5               g098(.a(new_n114), .b(new_n117), .c(new_n193), .d(new_n192), .out0(new_n194));
  inv040aa1n02x5               g099(.a(new_n122), .o1(new_n195));
  oaoi13aa1n09x5               g100(.a(new_n181), .b(new_n195), .c(new_n191), .d(new_n194), .o1(new_n196));
  nano23aa1n03x7               g101(.a(new_n140), .b(new_n135), .c(new_n141), .d(new_n134), .out0(new_n197));
  aoai13aa1n04x5               g102(.a(new_n163), .b(new_n149), .c(new_n197), .d(new_n133), .o1(new_n198));
  aoai13aa1n06x5               g103(.a(new_n182), .b(new_n180), .c(new_n198), .d(new_n164), .o1(new_n199));
  oai013aa1n02x4               g104(.a(new_n188), .b(new_n199), .c(new_n196), .d(new_n187), .o1(new_n200));
  xnrb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nano23aa1n06x5               g108(.a(new_n187), .b(new_n202), .c(new_n203), .d(new_n188), .out0(new_n204));
  oaih12aa1n02x5               g109(.a(new_n204), .b(new_n199), .c(new_n196), .o1(new_n205));
  tech160nm_fiaoi012aa1n04x5   g110(.a(new_n202), .b(new_n187), .c(new_n203), .o1(new_n206));
  nand42aa1n02x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nor042aa1n03x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n205), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand22aa1n03x5               g116(.a(new_n205), .b(new_n206), .o1(new_n212));
  nor042aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1n03x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n208), .c(new_n212), .d(new_n209), .o1(new_n216));
  inv040aa1n03x5               g121(.a(new_n206), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n209), .b(new_n217), .c(new_n185), .d(new_n204), .o1(new_n218));
  nona22aa1n02x4               g123(.a(new_n218), .b(new_n215), .c(new_n208), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n216), .b(new_n219), .o1(\s[20] ));
  norp02aa1n02x5               g125(.a(new_n199), .b(new_n196), .o1(new_n221));
  nano23aa1n06x5               g126(.a(new_n213), .b(new_n208), .c(new_n214), .d(new_n207), .out0(new_n222));
  nand02aa1n06x5               g127(.a(new_n222), .b(new_n204), .o1(new_n223));
  nona23aa1n09x5               g128(.a(new_n207), .b(new_n214), .c(new_n213), .d(new_n208), .out0(new_n224));
  tech160nm_fiaoi012aa1n04x5   g129(.a(new_n213), .b(new_n208), .c(new_n214), .o1(new_n225));
  tech160nm_fioai012aa1n05x5   g130(.a(new_n225), .b(new_n224), .c(new_n206), .o1(new_n226));
  oabi12aa1n03x5               g131(.a(new_n226), .b(new_n221), .c(new_n223), .out0(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n223), .o1(new_n234));
  aoai13aa1n02x7               g139(.a(new_n230), .b(new_n226), .c(new_n185), .d(new_n234), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n235), .b(new_n232), .c(new_n229), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n233), .b(new_n236), .o1(\s[22] ));
  inv000aa1n02x5               g142(.a(new_n225), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[21] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\a[22] ), .o1(new_n240));
  xroi22aa1d04x5               g145(.a(new_n239), .b(\b[20] ), .c(new_n240), .d(\b[21] ), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n238), .c(new_n222), .d(new_n217), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(new_n240), .b(new_n243), .c(new_n229), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  oai112aa1n03x5               g151(.a(new_n234), .b(new_n241), .c(new_n199), .d(new_n196), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .out0(new_n248));
  xobna2aa1n03x5               g153(.a(new_n248), .b(new_n247), .c(new_n246), .out0(\s[23] ));
  and002aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o(new_n250));
  xorc02aa1n12x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n244), .o1(new_n253));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n226), .d(new_n241), .o1(new_n255));
  aoai13aa1n02x7               g160(.a(new_n252), .b(new_n250), .c(new_n247), .d(new_n255), .o1(new_n256));
  aoi112aa1n02x7               g161(.a(new_n252), .b(new_n250), .c(new_n247), .d(new_n255), .o1(new_n257));
  norb02aa1n03x4               g162(.a(new_n256), .b(new_n257), .out0(\s[24] ));
  norb02aa1n02x7               g163(.a(new_n251), .b(new_n248), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n02x4               g165(.a(new_n260), .b(new_n241), .c(new_n222), .d(new_n204), .out0(new_n261));
  oaih12aa1n02x5               g166(.a(new_n261), .b(new_n199), .c(new_n196), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n263));
  oab012aa1n02x4               g168(.a(new_n263), .b(\a[24] ), .c(\b[23] ), .out0(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n260), .c(new_n242), .d(new_n244), .o1(new_n265));
  nanb02aa1n02x5               g170(.a(new_n265), .b(new_n262), .out0(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  tech160nm_fixorc02aa1n02p5x5 g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n12x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n272));
  aoai13aa1n02x7               g177(.a(new_n269), .b(new_n265), .c(new_n185), .d(new_n261), .o1(new_n273));
  nona22aa1n02x4               g178(.a(new_n273), .b(new_n271), .c(new_n268), .out0(new_n274));
  nanp02aa1n02x5               g179(.a(new_n272), .b(new_n274), .o1(\s[26] ));
  and002aa1n06x5               g180(.a(new_n270), .b(new_n269), .o(new_n276));
  nano32aa1n03x7               g181(.a(new_n223), .b(new_n276), .c(new_n241), .d(new_n259), .out0(new_n277));
  oai012aa1n06x5               g182(.a(new_n277), .b(new_n199), .c(new_n196), .o1(new_n278));
  orn002aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .o(new_n279));
  oao003aa1n02x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n279), .carry(new_n280));
  aobi12aa1n06x5               g185(.a(new_n280), .b(new_n265), .c(new_n276), .out0(new_n281));
  nor042aa1n03x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n283), .b(new_n282), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n281), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  aobi12aa1n02x7               g192(.a(new_n283), .b(new_n281), .c(new_n278), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n259), .b(new_n253), .c(new_n226), .d(new_n241), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n276), .o1(new_n291));
  aoai13aa1n04x5               g196(.a(new_n280), .b(new_n291), .c(new_n290), .d(new_n264), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n283), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n293));
  aoi012aa1n03x5               g198(.a(new_n287), .b(new_n293), .c(new_n286), .o1(new_n294));
  norp02aa1n03x5               g199(.a(new_n294), .b(new_n289), .o1(\s[28] ));
  nano22aa1n02x4               g200(.a(new_n287), .b(new_n286), .c(new_n283), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  aoi012aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n06x5               g205(.a(new_n296), .b(new_n281), .c(new_n278), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[29] ));
  xorb03aa1n02x5               g208(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g209(.a(new_n284), .b(new_n299), .c(new_n287), .out0(new_n305));
  aoai13aa1n02x7               g210(.a(new_n305), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[29] ), .b(\a[30] ), .out0(new_n308));
  aoi012aa1n03x5               g213(.a(new_n308), .b(new_n306), .c(new_n307), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n305), .b(new_n281), .c(new_n278), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n307), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[30] ));
  norb03aa1n02x5               g217(.a(new_n296), .b(new_n308), .c(new_n299), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n292), .c(new_n185), .d(new_n277), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[30] ), .b(\a[31] ), .out0(new_n316));
  aoi012aa1n03x5               g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n03x5               g222(.a(new_n313), .b(new_n281), .c(new_n278), .out0(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[31] ));
  xnrb03aa1n02x5               g225(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g226(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n02x5               g229(.a(\a[5] ), .b(\b[4] ), .c(new_n191), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(new_n325), .b(new_n117), .out0(\s[6] ));
  inv000aa1d42x5               g231(.a(new_n193), .o1(new_n327));
  aob012aa1n02x5               g232(.a(new_n116), .b(new_n325), .c(new_n117), .out0(new_n328));
  xnrc02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(\s[7] ));
  inv000aa1d42x5               g234(.a(new_n111), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n192), .b(new_n330), .c(new_n328), .d(new_n327), .o1(new_n331));
  aoi112aa1n02x5               g236(.a(new_n192), .b(new_n330), .c(new_n328), .d(new_n327), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n331), .b(new_n332), .out0(\s[8] ));
  xnbna2aa1n03x5               g238(.a(new_n179), .b(new_n124), .c(new_n97), .out0(\s[9] ));
endmodule


