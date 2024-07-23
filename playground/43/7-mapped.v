// Benchmark "adder" written by ABC on Thu Jul 18 10:01:33 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n319,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nand42aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor042aa1n04x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand42aa1n10x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n09x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .d(new_n100), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[5] ), .o1(new_n105));
  inv040aa1d28x5               g010(.a(\b[4] ), .o1(new_n106));
  nand02aa1d20x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  oaoi03aa1n06x5               g012(.a(\a[6] ), .b(\b[5] ), .c(new_n107), .o1(new_n108));
  oai022aa1n03x5               g013(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n109));
  aoi022aa1n12x5               g014(.a(new_n104), .b(new_n108), .c(new_n100), .d(new_n109), .o1(new_n110));
  orn002aa1n24x5               g015(.a(\a[2] ), .b(\b[1] ), .o(new_n111));
  nand22aa1n12x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nand22aa1n09x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aob012aa1n12x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .out0(new_n114));
  nor042aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nand22aa1n06x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  norb02aa1n06x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nor042aa1n06x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand02aa1d04x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  norb02aa1n06x5               g024(.a(new_n119), .b(new_n118), .out0(new_n120));
  nand23aa1n09x5               g025(.a(new_n114), .b(new_n117), .c(new_n120), .o1(new_n121));
  tech160nm_fioai012aa1n03p5x5 g026(.a(new_n116), .b(new_n118), .c(new_n115), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  nor042aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  nanb02aa1n02x5               g029(.a(new_n124), .b(new_n123), .out0(new_n125));
  tech160nm_finand02aa1n05x5   g030(.a(\b[4] ), .b(\a[5] ), .o1(new_n126));
  nanp02aa1n03x5               g031(.a(new_n107), .b(new_n126), .o1(new_n127));
  nona22aa1n03x5               g032(.a(new_n104), .b(new_n125), .c(new_n127), .out0(new_n128));
  aoai13aa1n12x5               g033(.a(new_n110), .b(new_n128), .c(new_n121), .d(new_n122), .o1(new_n129));
  tech160nm_fioai012aa1n05x5   g034(.a(new_n98), .b(new_n129), .c(new_n99), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n03x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n130), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor022aa1n06x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoai13aa1n02x7               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  xorc02aa1n02x5               g045(.a(\a[10] ), .b(\b[9] ), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n130), .b(new_n141), .o1(new_n142));
  nanb02aa1d24x5               g047(.a(new_n134), .b(new_n135), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nanp03aa1n02x5               g049(.a(new_n142), .b(new_n140), .c(new_n144), .o1(new_n145));
  nona22aa1n02x4               g050(.a(new_n145), .b(new_n138), .c(new_n134), .out0(new_n146));
  nanp02aa1n03x5               g051(.a(new_n139), .b(new_n146), .o1(\s[12] ));
  nano23aa1n06x5               g052(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n148));
  inv000aa1d42x5               g053(.a(\b[9] ), .o1(new_n149));
  oao003aa1n02x5               g054(.a(new_n97), .b(new_n149), .c(new_n99), .carry(new_n150));
  oai012aa1n02x5               g055(.a(new_n137), .b(new_n136), .c(new_n134), .o1(new_n151));
  aobi12aa1n06x5               g056(.a(new_n151), .b(new_n148), .c(new_n150), .out0(new_n152));
  norb02aa1n02x5               g057(.a(new_n98), .b(new_n99), .out0(new_n153));
  and003aa1n02x5               g058(.a(new_n148), .b(new_n141), .c(new_n153), .o(new_n154));
  nanp02aa1n03x5               g059(.a(new_n129), .b(new_n154), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n152), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand02aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  tech160nm_fiaoi012aa1n05x5   g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nand22aa1n09x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanb02aa1d24x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  oaoi03aa1n03x5               g070(.a(new_n97), .b(new_n149), .c(new_n99), .o1(new_n166));
  oai013aa1n03x5               g071(.a(new_n151), .b(new_n166), .c(new_n143), .d(new_n138), .o1(new_n167));
  nor022aa1n06x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nano23aa1n06x5               g074(.a(new_n158), .b(new_n168), .c(new_n169), .d(new_n159), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n167), .c(new_n129), .d(new_n154), .o1(new_n171));
  tech160nm_fioai012aa1n05x5   g076(.a(new_n169), .b(new_n168), .c(new_n158), .o1(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n165), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  nand42aa1n02x5               g078(.a(new_n171), .b(new_n172), .o1(new_n174));
  nor042aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n06x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n162), .c(new_n174), .d(new_n165), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(new_n174), .b(new_n165), .o1(new_n179));
  nona22aa1n02x4               g084(.a(new_n179), .b(new_n177), .c(new_n162), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n178), .o1(\s[16] ));
  oai012aa1n02x5               g086(.a(new_n176), .b(new_n175), .c(new_n162), .o1(new_n182));
  oai013aa1n03x5               g087(.a(new_n182), .b(new_n172), .c(new_n164), .d(new_n177), .o1(new_n183));
  nona22aa1n09x5               g088(.a(new_n170), .b(new_n177), .c(new_n164), .out0(new_n184));
  aoib12aa1n12x5               g089(.a(new_n183), .b(new_n167), .c(new_n184), .out0(new_n185));
  nano32aa1n06x5               g090(.a(new_n184), .b(new_n148), .c(new_n153), .d(new_n141), .out0(new_n186));
  nand02aa1d08x5               g091(.a(new_n129), .b(new_n186), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n187), .c(new_n185), .out0(\s[17] ));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(\b[16] ), .b(new_n190), .out0(new_n191));
  inv030aa1n03x5               g096(.a(new_n183), .o1(new_n192));
  tech160nm_fioai012aa1n05x5   g097(.a(new_n192), .b(new_n152), .c(new_n184), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n188), .b(new_n193), .c(new_n129), .d(new_n186), .o1(new_n194));
  xnrc02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n194), .c(new_n191), .out0(\s[18] ));
  inv000aa1d42x5               g101(.a(\a[18] ), .o1(new_n197));
  xroi22aa1d04x5               g102(.a(new_n190), .b(\b[16] ), .c(new_n197), .d(\b[17] ), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n193), .c(new_n129), .d(new_n186), .o1(new_n199));
  oai022aa1n04x7               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  oaib12aa1n12x5               g105(.a(new_n200), .b(new_n197), .c(\b[17] ), .out0(new_n201));
  nor002aa1n12x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n199), .c(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g112(.a(new_n199), .b(new_n201), .o1(new_n208));
  nor002aa1n16x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1n06x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n202), .c(new_n208), .d(new_n205), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(new_n208), .b(new_n205), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n213), .b(new_n211), .c(new_n202), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n212), .o1(\s[20] ));
  nona23aa1d18x5               g120(.a(new_n210), .b(new_n203), .c(new_n202), .d(new_n209), .out0(new_n216));
  oai012aa1n12x5               g121(.a(new_n210), .b(new_n209), .c(new_n202), .o1(new_n217));
  oai012aa1d24x5               g122(.a(new_n217), .b(new_n216), .c(new_n201), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nano23aa1n09x5               g124(.a(new_n202), .b(new_n209), .c(new_n210), .d(new_n203), .out0(new_n220));
  nanb03aa1n02x5               g125(.a(new_n195), .b(new_n220), .c(new_n188), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n219), .b(new_n221), .c(new_n187), .d(new_n185), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  tech160nm_fixnrc02aa1n05x5   g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n228));
  nand42aa1n02x5               g133(.a(new_n222), .b(new_n226), .o1(new_n229));
  nona22aa1n03x5               g134(.a(new_n229), .b(new_n227), .c(new_n224), .out0(new_n230));
  nanp02aa1n03x5               g135(.a(new_n230), .b(new_n228), .o1(\s[22] ));
  nor042aa1n06x5               g136(.a(new_n227), .b(new_n225), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\a[22] ), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\b[21] ), .o1(new_n234));
  oao003aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n224), .carry(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n218), .c(new_n232), .o1(new_n236));
  nand03aa1n02x5               g141(.a(new_n198), .b(new_n232), .c(new_n220), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n236), .b(new_n237), .c(new_n187), .d(new_n185), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  tech160nm_fixorc02aa1n05x5   g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  tech160nm_fixnrc02aa1n04x5   g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n243));
  tech160nm_finand02aa1n03p5x5 g148(.a(new_n238), .b(new_n241), .o1(new_n244));
  nona22aa1n02x5               g149(.a(new_n244), .b(new_n242), .c(new_n240), .out0(new_n245));
  nanp02aa1n03x5               g150(.a(new_n245), .b(new_n243), .o1(\s[24] ));
  norb02aa1n03x5               g151(.a(new_n241), .b(new_n242), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  nano32aa1n02x4               g153(.a(new_n248), .b(new_n198), .c(new_n232), .d(new_n220), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n193), .c(new_n129), .d(new_n186), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n217), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n232), .b(new_n252), .c(new_n220), .d(new_n251), .o1(new_n253));
  inv000aa1n02x5               g158(.a(new_n235), .o1(new_n254));
  orn002aa1n03x5               g159(.a(\a[23] ), .b(\b[22] ), .o(new_n255));
  oaoi03aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n255), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n248), .c(new_n253), .d(new_n254), .o1(new_n258));
  nanb02aa1n03x5               g163(.a(new_n258), .b(new_n250), .out0(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n259), .b(new_n262), .o1(new_n266));
  nona22aa1n02x4               g171(.a(new_n266), .b(new_n264), .c(new_n261), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  and002aa1n12x5               g173(.a(new_n263), .b(new_n262), .o(new_n269));
  nanp02aa1n02x5               g174(.a(new_n258), .b(new_n269), .o1(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n271), .b(\b[25] ), .c(\a[26] ), .out0(new_n272));
  nano22aa1n03x7               g177(.a(new_n237), .b(new_n269), .c(new_n247), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n193), .c(new_n129), .d(new_n186), .o1(new_n274));
  nand23aa1n04x5               g179(.a(new_n270), .b(new_n274), .c(new_n272), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n277), .c(new_n275), .d(new_n278), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n247), .b(new_n235), .c(new_n218), .d(new_n232), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n269), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n272), .b(new_n282), .c(new_n281), .d(new_n257), .o1(new_n283));
  aobi12aa1n09x5               g188(.a(new_n273), .b(new_n187), .c(new_n185), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n278), .b(new_n283), .c(new_n284), .o1(new_n285));
  nona22aa1n02x4               g190(.a(new_n285), .b(new_n279), .c(new_n277), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n280), .b(new_n286), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n278), .b(new_n279), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n283), .c(new_n284), .o1(new_n289));
  inv000aa1n03x5               g194(.a(new_n277), .o1(new_n290));
  oaoi03aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nona22aa1n02x4               g197(.a(new_n289), .b(new_n291), .c(new_n292), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n292), .b(new_n291), .c(new_n275), .d(new_n288), .o1(new_n294));
  nanp02aa1n03x5               g199(.a(new_n294), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n278), .b(new_n292), .c(new_n279), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .carry(new_n298));
  oaoi03aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n298), .o1(new_n299));
  tech160nm_fixorc02aa1n03p5x5 g204(.a(\a[30] ), .b(\b[29] ), .out0(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n299), .c(new_n275), .d(new_n297), .o1(new_n302));
  oaih12aa1n02x5               g207(.a(new_n297), .b(new_n283), .c(new_n284), .o1(new_n303));
  nona22aa1n03x5               g208(.a(new_n303), .b(new_n299), .c(new_n301), .out0(new_n304));
  nanp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  nanp02aa1n02x5               g210(.a(new_n299), .b(new_n300), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(\b[29] ), .c(\a[30] ), .o1(new_n307));
  nano23aa1n02x4               g212(.a(new_n292), .b(new_n279), .c(new_n300), .d(new_n278), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n283), .c(new_n284), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nona22aa1n03x5               g215(.a(new_n309), .b(new_n310), .c(new_n307), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n310), .b(new_n307), .c(new_n275), .d(new_n308), .o1(new_n312));
  nanp02aa1n03x5               g217(.a(new_n312), .b(new_n311), .o1(\s[31] ));
  xorb03aa1n02x5               g218(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g219(.a(new_n118), .b(new_n114), .c(new_n119), .o1(new_n315));
  xnrc02aa1n02x5               g220(.a(new_n315), .b(new_n117), .out0(\s[4] ));
  xobna2aa1n03x5               g221(.a(new_n127), .b(new_n121), .c(new_n122), .out0(\s[5] ));
  nanp02aa1n02x5               g222(.a(new_n121), .b(new_n122), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(new_n105), .b(new_n106), .c(new_n318), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g225(.a(\a[6] ), .b(\b[5] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g227(.a(new_n102), .b(new_n321), .c(new_n103), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


