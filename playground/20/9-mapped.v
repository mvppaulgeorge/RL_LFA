// Benchmark "adder" written by ABC on Wed Jul 17 22:15:28 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n319, new_n320,
    new_n321, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n12x5               g004(.a(\b[1] ), .b(\a[2] ), .o(new_n100));
  nand22aa1n12x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oab012aa1n09x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .out0(new_n103));
  tech160nm_fixorc02aa1n04x5   g008(.a(\a[4] ), .b(\b[3] ), .out0(new_n104));
  nor042aa1d18x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nanp03aa1n06x5               g012(.a(new_n103), .b(new_n104), .c(new_n107), .o1(new_n108));
  inv040aa1n02x5               g013(.a(new_n105), .o1(new_n109));
  oao003aa1n09x5               g014(.a(\a[4] ), .b(\b[3] ), .c(new_n109), .carry(new_n110));
  nor002aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand42aa1n10x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n10x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n09x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  tech160nm_fixorc02aa1n05x5   g020(.a(\a[6] ), .b(\b[5] ), .out0(new_n116));
  nor022aa1n16x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand02aa1d06x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  norb02aa1n03x5               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nand03aa1n02x5               g024(.a(new_n115), .b(new_n116), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  inv040aa1d32x5               g026(.a(\b[5] ), .o1(new_n122));
  oao003aa1n03x5               g027(.a(new_n121), .b(new_n122), .c(new_n117), .carry(new_n123));
  inv000aa1n02x5               g028(.a(new_n113), .o1(new_n124));
  oaoi03aa1n03x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  tech160nm_fiaoi012aa1n03p5x5 g030(.a(new_n125), .b(new_n115), .c(new_n123), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n120), .c(new_n108), .d(new_n110), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(new_n127), .b(new_n128), .o1(new_n129));
  nor022aa1n08x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  tech160nm_finand02aa1n03p5x5 g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanb02aa1n12x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  xobna2aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n99), .out0(\s[10] ));
  norb02aa1n02x5               g038(.a(new_n128), .b(new_n132), .out0(new_n134));
  oaoi03aa1n02x5               g039(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n135));
  nor042aa1n09x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  tech160nm_finand02aa1n03p5x5 g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n06x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n135), .c(new_n127), .d(new_n134), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n127), .d(new_n134), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  inv040aa1n03x5               g046(.a(new_n136), .o1(new_n142));
  nor022aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n06x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n139), .c(new_n142), .out0(\s[12] ));
  aoai13aa1n02x5               g051(.a(new_n131), .b(new_n130), .c(new_n97), .d(new_n98), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n144), .b(new_n137), .c(new_n136), .d(new_n143), .out0(new_n148));
  oaoi03aa1n02x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n142), .o1(new_n149));
  oabi12aa1n02x7               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(new_n150));
  nona23aa1d18x5               g055(.a(new_n138), .b(new_n128), .c(new_n145), .d(new_n132), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n127), .d(new_n152), .o1(new_n155));
  aoi112aa1n02x5               g060(.a(new_n150), .b(new_n154), .c(new_n127), .d(new_n152), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  nor042aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  inv040aa1n03x5               g063(.a(new_n158), .o1(new_n159));
  tech160nm_fixnrc02aa1n04x5   g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  xobna2aa1n03x5               g065(.a(new_n160), .b(new_n155), .c(new_n159), .out0(\s[14] ));
  nona32aa1n09x5               g066(.a(new_n127), .b(new_n160), .c(new_n153), .d(new_n151), .out0(new_n162));
  nor042aa1n06x5               g067(.a(new_n160), .b(new_n153), .o1(new_n163));
  oaoi03aa1n09x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n164));
  tech160nm_fiaoi012aa1n05x5   g069(.a(new_n164), .b(new_n150), .c(new_n163), .o1(new_n165));
  xnrc02aa1n12x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n162), .c(new_n165), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  tech160nm_fiao0012aa1n02p5x5 g073(.a(new_n166), .b(new_n162), .c(new_n165), .o(new_n169));
  tech160nm_fixnrc02aa1n05x5   g074(.a(\b[15] ), .b(\a[16] ), .out0(new_n170));
  oaib12aa1n03x5               g075(.a(new_n170), .b(new_n168), .c(new_n169), .out0(new_n171));
  nona22aa1n02x4               g076(.a(new_n169), .b(new_n170), .c(new_n168), .out0(new_n172));
  nanp02aa1n03x5               g077(.a(new_n171), .b(new_n172), .o1(\s[16] ));
  inv040aa1d30x5               g078(.a(\a[17] ), .o1(new_n174));
  nor042aa1n04x5               g079(.a(new_n170), .b(new_n166), .o1(new_n175));
  nano22aa1n06x5               g080(.a(new_n151), .b(new_n163), .c(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(\a[16] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\b[15] ), .o1(new_n178));
  oao003aa1n12x5               g083(.a(new_n177), .b(new_n178), .c(new_n168), .carry(new_n179));
  nano23aa1n02x4               g084(.a(new_n136), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n163), .b(new_n149), .c(new_n180), .d(new_n135), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n164), .o1(new_n182));
  inv000aa1n02x5               g087(.a(new_n175), .o1(new_n183));
  aoi012aa1n02x7               g088(.a(new_n183), .b(new_n181), .c(new_n182), .o1(new_n184));
  aoi112aa1n09x5               g089(.a(new_n184), .b(new_n179), .c(new_n127), .d(new_n176), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(new_n174), .out0(\s[17] ));
  oaoi03aa1n03x5               g091(.a(\a[17] ), .b(\b[16] ), .c(new_n185), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  xroi22aa1d06x4               g094(.a(new_n174), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n190));
  inv000aa1n03x5               g095(.a(new_n190), .o1(new_n191));
  oai022aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n09x5               g097(.a(new_n192), .b(new_n189), .c(\b[17] ), .out0(new_n193));
  tech160nm_fioai012aa1n05x5   g098(.a(new_n193), .b(new_n185), .c(new_n191), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand42aa1n06x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nor022aa1n16x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand42aa1n06x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n194), .d(new_n200), .o1(new_n204));
  nanp02aa1n06x5               g109(.a(new_n127), .b(new_n176), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n179), .o1(new_n206));
  oai112aa1n06x5               g111(.a(new_n205), .b(new_n206), .c(new_n183), .d(new_n165), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(\b[16] ), .b(new_n174), .out0(new_n208));
  oaoi03aa1n12x5               g113(.a(\a[18] ), .b(\b[17] ), .c(new_n208), .o1(new_n209));
  aoai13aa1n02x7               g114(.a(new_n200), .b(new_n209), .c(new_n207), .d(new_n190), .o1(new_n210));
  nona22aa1n03x5               g115(.a(new_n210), .b(new_n203), .c(new_n197), .out0(new_n211));
  nanp02aa1n03x5               g116(.a(new_n204), .b(new_n211), .o1(\s[20] ));
  nona23aa1n06x5               g117(.a(new_n202), .b(new_n198), .c(new_n197), .d(new_n201), .out0(new_n213));
  oa0012aa1n02x5               g118(.a(new_n202), .b(new_n201), .c(new_n197), .o(new_n214));
  inv030aa1n03x5               g119(.a(new_n214), .o1(new_n215));
  oai012aa1n18x5               g120(.a(new_n215), .b(new_n213), .c(new_n193), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nano23aa1n09x5               g122(.a(new_n197), .b(new_n201), .c(new_n202), .d(new_n198), .out0(new_n218));
  nand22aa1n09x5               g123(.a(new_n190), .b(new_n218), .o1(new_n219));
  tech160nm_fioai012aa1n05x5   g124(.a(new_n217), .b(new_n185), .c(new_n219), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n219), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n224), .b(new_n216), .c(new_n207), .d(new_n227), .o1(new_n228));
  nona22aa1n02x4               g133(.a(new_n228), .b(new_n225), .c(new_n222), .out0(new_n229));
  nanp02aa1n03x5               g134(.a(new_n226), .b(new_n229), .o1(\s[22] ));
  norp02aa1n12x5               g135(.a(new_n225), .b(new_n223), .o1(new_n231));
  nano22aa1n03x7               g136(.a(new_n191), .b(new_n231), .c(new_n218), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\a[22] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[21] ), .o1(new_n235));
  oao003aa1n06x5               g140(.a(new_n234), .b(new_n235), .c(new_n222), .carry(new_n236));
  tech160nm_fiaoi012aa1n03p5x5 g141(.a(new_n236), .b(new_n216), .c(new_n231), .o1(new_n237));
  tech160nm_fioai012aa1n05x5   g142(.a(new_n237), .b(new_n185), .c(new_n233), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n12x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n237), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n241), .b(new_n244), .c(new_n207), .d(new_n232), .o1(new_n245));
  nona22aa1n02x4               g150(.a(new_n245), .b(new_n242), .c(new_n240), .out0(new_n246));
  nanp02aa1n03x5               g151(.a(new_n243), .b(new_n246), .o1(\s[24] ));
  norb02aa1n02x7               g152(.a(new_n241), .b(new_n242), .out0(new_n248));
  inv020aa1n04x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n03x7               g154(.a(new_n249), .b(new_n190), .c(new_n231), .d(new_n218), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n231), .b(new_n214), .c(new_n218), .d(new_n209), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n236), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  aob012aa1n02x5               g159(.a(new_n254), .b(\b[23] ), .c(\a[24] ), .out0(new_n255));
  aoai13aa1n12x5               g160(.a(new_n255), .b(new_n249), .c(new_n252), .d(new_n253), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fioai012aa1n05x5   g162(.a(new_n257), .b(new_n185), .c(new_n251), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  tech160nm_fixnrc02aa1n04x5   g166(.a(\b[25] ), .b(\a[26] ), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n02x7               g168(.a(new_n261), .b(new_n256), .c(new_n207), .d(new_n250), .o1(new_n264));
  nona22aa1n02x4               g169(.a(new_n264), .b(new_n262), .c(new_n260), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n263), .b(new_n265), .o1(\s[26] ));
  norb02aa1n06x4               g171(.a(new_n261), .b(new_n262), .out0(new_n267));
  nano23aa1n06x5               g172(.a(new_n219), .b(new_n249), .c(new_n267), .d(new_n231), .out0(new_n268));
  inv020aa1n03x5               g173(.a(new_n268), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .o1(new_n270));
  oai022aa1n02x5               g175(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n271));
  aoi022aa1n06x5               g176(.a(new_n256), .b(new_n267), .c(new_n270), .d(new_n271), .o1(new_n272));
  oai012aa1n06x5               g177(.a(new_n272), .b(new_n185), .c(new_n269), .o1(new_n273));
  xorb03aa1n03x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n275), .c(new_n273), .d(new_n276), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n248), .b(new_n236), .c(new_n216), .d(new_n231), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n267), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n271), .b(new_n270), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n281), .b(new_n280), .c(new_n279), .d(new_n255), .o1(new_n282));
  aoai13aa1n02x5               g187(.a(new_n276), .b(new_n282), .c(new_n207), .d(new_n268), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n277), .c(new_n275), .out0(new_n284));
  nanp02aa1n03x5               g189(.a(new_n278), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n276), .b(new_n277), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n207), .d(new_n268), .o1(new_n287));
  inv000aa1n03x5               g192(.a(new_n275), .o1(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nona22aa1n02x4               g195(.a(new_n287), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n290), .b(new_n289), .c(new_n273), .d(new_n286), .o1(new_n292));
  nanp02aa1n03x5               g197(.a(new_n292), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n276), .b(new_n290), .c(new_n277), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .carry(new_n296));
  oaoi03aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .o1(new_n297));
  tech160nm_fixorc02aa1n03p5x5 g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n273), .d(new_n295), .o1(new_n300));
  aoai13aa1n02x5               g205(.a(new_n295), .b(new_n282), .c(new_n207), .d(new_n268), .o1(new_n301));
  nona22aa1n02x4               g206(.a(new_n301), .b(new_n297), .c(new_n299), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  nano23aa1n02x4               g208(.a(new_n290), .b(new_n277), .c(new_n298), .d(new_n276), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n282), .c(new_n207), .d(new_n268), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n297), .b(new_n298), .o1(new_n306));
  oai012aa1n02x5               g211(.a(new_n306), .b(\b[29] ), .c(\a[30] ), .o1(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nona22aa1n02x4               g213(.a(new_n305), .b(new_n307), .c(new_n308), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n307), .c(new_n273), .d(new_n304), .o1(new_n310));
  nanp02aa1n03x5               g215(.a(new_n310), .b(new_n309), .o1(\s[31] ));
  xobna2aa1n03x5               g216(.a(new_n103), .b(new_n106), .c(new_n109), .out0(\s[3] ));
  oai012aa1n02x5               g217(.a(new_n106), .b(new_n103), .c(new_n105), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g219(.a(new_n119), .b(new_n108), .c(new_n110), .out0(\s[5] ));
  nanp03aa1n02x5               g220(.a(new_n108), .b(new_n110), .c(new_n119), .o1(new_n316));
  xobna2aa1n03x5               g221(.a(new_n116), .b(new_n316), .c(new_n118), .out0(\s[6] ));
  norb02aa1n02x5               g222(.a(new_n114), .b(new_n113), .out0(new_n318));
  nanp02aa1n02x5               g223(.a(new_n316), .b(new_n118), .o1(new_n319));
  nanp02aa1n02x5               g224(.a(new_n319), .b(new_n116), .o1(new_n320));
  oai112aa1n02x5               g225(.a(new_n320), .b(new_n318), .c(new_n122), .d(new_n121), .o1(new_n321));
  oaoi13aa1n02x5               g226(.a(new_n318), .b(new_n320), .c(new_n121), .d(new_n122), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n321), .b(new_n322), .out0(\s[7] ));
  norb02aa1n02x5               g228(.a(new_n112), .b(new_n111), .out0(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n324), .b(new_n321), .c(new_n124), .out0(\s[8] ));
  xorb03aa1n02x5               g230(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


