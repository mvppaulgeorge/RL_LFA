// Benchmark "adder" written by ABC on Wed Jul 17 16:53:02 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n325, new_n326, new_n328, new_n330, new_n332, new_n333, new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n03p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand22aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n06x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n10x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n24x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  tech160nm_fioai012aa1n04x5   g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  tech160nm_fioai012aa1n03p5x5 g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanb02aa1d36x5               g017(.a(new_n111), .b(new_n112), .out0(new_n113));
  nor002aa1n20x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  tech160nm_finand02aa1n05x5   g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nand02aa1n06x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norp02aa1n12x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanb02aa1n12x5               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  xorc02aa1n06x5               g024(.a(\a[5] ), .b(\b[4] ), .out0(new_n120));
  nano23aa1n06x5               g025(.a(new_n113), .b(new_n119), .c(new_n120), .d(new_n116), .out0(new_n121));
  inv000aa1d42x5               g026(.a(new_n111), .o1(new_n122));
  inv000aa1d42x5               g027(.a(new_n112), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n114), .o1(new_n124));
  nor002aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n115), .b(new_n118), .c(new_n125), .d(new_n117), .o1(new_n126));
  aoai13aa1n04x5               g031(.a(new_n122), .b(new_n123), .c(new_n126), .d(new_n124), .o1(new_n127));
  tech160nm_fixorc02aa1n03p5x5 g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n121), .d(new_n110), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n97), .b(new_n129), .c(new_n99), .out0(\s[10] ));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor002aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oaih22aa1d12x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n129), .b(new_n135), .o1(new_n136));
  nand42aa1n03x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  inv000aa1n03x5               g042(.a(new_n103), .o1(new_n138));
  nano23aa1n09x5               g043(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n139));
  aobi12aa1n06x5               g044(.a(new_n109), .b(new_n139), .c(new_n138), .out0(new_n140));
  nona23aa1n03x5               g045(.a(new_n120), .b(new_n116), .c(new_n113), .d(new_n119), .out0(new_n141));
  nand02aa1n02x5               g046(.a(new_n126), .b(new_n124), .o1(new_n142));
  aoi012aa1n06x5               g047(.a(new_n111), .b(new_n142), .c(new_n112), .o1(new_n143));
  oai012aa1n09x5               g048(.a(new_n143), .b(new_n140), .c(new_n141), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n137), .b(new_n134), .c(new_n144), .d(new_n128), .o1(new_n145));
  nano22aa1n03x7               g050(.a(new_n132), .b(new_n137), .c(new_n131), .out0(new_n146));
  aoi022aa1n02x5               g051(.a(new_n145), .b(new_n133), .c(new_n136), .d(new_n146), .o1(\s[11] ));
  aoi022aa1n02x5               g052(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n148));
  tech160nm_fiaoi012aa1n05x5   g053(.a(new_n132), .b(new_n136), .c(new_n148), .o1(new_n149));
  nor002aa1n03x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nand42aa1n06x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  norb02aa1n06x4               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  xnrc02aa1n02x5               g057(.a(new_n149), .b(new_n152), .out0(\s[12] ));
  nano23aa1n06x5               g058(.a(new_n150), .b(new_n132), .c(new_n151), .d(new_n131), .out0(new_n154));
  and003aa1n02x5               g059(.a(new_n154), .b(new_n128), .c(new_n97), .o(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n127), .c(new_n121), .d(new_n110), .o1(new_n156));
  oa0012aa1n06x5               g061(.a(new_n151), .b(new_n150), .c(new_n132), .o(new_n157));
  aoi013aa1n06x4               g062(.a(new_n157), .b(new_n146), .c(new_n152), .d(new_n134), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n156), .b(new_n158), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand02aa1n03x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nona23aa1n02x4               g069(.a(new_n148), .b(new_n151), .c(new_n150), .d(new_n132), .out0(new_n165));
  oabi12aa1n02x5               g070(.a(new_n157), .b(new_n165), .c(new_n135), .out0(new_n166));
  nor002aa1n03x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nanp02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nano23aa1n06x5               g073(.a(new_n161), .b(new_n167), .c(new_n168), .d(new_n162), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n166), .c(new_n144), .d(new_n155), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n167), .b(new_n161), .c(new_n168), .o1(new_n171));
  nor002aa1n12x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n20x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n170), .c(new_n171), .out0(\s[15] ));
  tech160nm_finand02aa1n05x5   g080(.a(new_n170), .b(new_n171), .o1(new_n176));
  nor002aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n172), .c(new_n176), .d(new_n173), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n176), .b(new_n174), .o1(new_n181));
  nona22aa1n02x4               g086(.a(new_n181), .b(new_n179), .c(new_n172), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n180), .o1(\s[16] ));
  nano23aa1n03x5               g088(.a(new_n172), .b(new_n177), .c(new_n178), .d(new_n173), .out0(new_n184));
  nand02aa1d04x5               g089(.a(new_n184), .b(new_n169), .o1(new_n185));
  nano32aa1n09x5               g090(.a(new_n185), .b(new_n154), .c(new_n128), .d(new_n97), .out0(new_n186));
  aoai13aa1n09x5               g091(.a(new_n186), .b(new_n127), .c(new_n121), .d(new_n110), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n172), .o1(new_n188));
  aoai13aa1n03x5               g093(.a(new_n173), .b(new_n167), .c(new_n161), .d(new_n168), .o1(new_n189));
  aoi022aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nor042aa1n02x5               g095(.a(new_n190), .b(new_n177), .o1(new_n191));
  oai012aa1n12x5               g096(.a(new_n191), .b(new_n158), .c(new_n185), .o1(new_n192));
  inv030aa1n06x5               g097(.a(new_n192), .o1(new_n193));
  nor042aa1n06x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nand42aa1n03x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n187), .c(new_n193), .out0(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  nand02aa1d08x5               g103(.a(new_n187), .b(new_n193), .o1(new_n199));
  tech160nm_fiaoi012aa1n05x5   g104(.a(new_n194), .b(new_n199), .c(new_n196), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(new_n198), .out0(\s[18] ));
  nor042aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand42aa1n06x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nano23aa1n09x5               g108(.a(new_n194), .b(new_n202), .c(new_n203), .d(new_n195), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n192), .c(new_n144), .d(new_n186), .o1(new_n205));
  oa0012aa1n02x5               g110(.a(new_n203), .b(new_n202), .c(new_n194), .o(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  nor002aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nand42aa1n04x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  norb02aa1n06x4               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n205), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g117(.a(new_n205), .b(new_n207), .o1(new_n213));
  nor042aa1n09x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand22aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanb02aa1n03x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n208), .c(new_n213), .d(new_n209), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n210), .b(new_n206), .c(new_n199), .d(new_n204), .o1(new_n218));
  nona22aa1n03x5               g123(.a(new_n218), .b(new_n216), .c(new_n208), .out0(new_n219));
  nanp02aa1n03x5               g124(.a(new_n217), .b(new_n219), .o1(\s[20] ));
  nanb03aa1n12x5               g125(.a(new_n214), .b(new_n215), .c(new_n209), .out0(new_n221));
  orn002aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .o(new_n222));
  oai112aa1n06x5               g127(.a(new_n222), .b(new_n203), .c(new_n202), .d(new_n194), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n214), .o1(new_n224));
  aob012aa1n06x5               g129(.a(new_n224), .b(new_n208), .c(new_n215), .out0(new_n225));
  oabi12aa1n18x5               g130(.a(new_n225), .b(new_n223), .c(new_n221), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  nanb03aa1n06x5               g132(.a(new_n216), .b(new_n204), .c(new_n210), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n227), .b(new_n228), .c(new_n187), .d(new_n193), .o1(new_n229));
  nor042aa1n06x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  aoib12aa1n02x5               g137(.a(new_n232), .b(new_n199), .c(new_n228), .out0(new_n233));
  aoi022aa1n02x5               g138(.a(new_n233), .b(new_n227), .c(new_n229), .d(new_n232), .o1(\s[21] ));
  nor042aa1n04x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  nanb02aa1n02x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n230), .c(new_n229), .d(new_n232), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(new_n229), .b(new_n232), .o1(new_n239));
  nona22aa1n02x4               g144(.a(new_n239), .b(new_n237), .c(new_n230), .out0(new_n240));
  nanp02aa1n02x5               g145(.a(new_n240), .b(new_n238), .o1(\s[22] ));
  nano23aa1n06x5               g146(.a(new_n230), .b(new_n235), .c(new_n236), .d(new_n231), .out0(new_n242));
  nanb02aa1n02x5               g147(.a(new_n228), .b(new_n242), .out0(new_n243));
  oa0012aa1n02x5               g148(.a(new_n236), .b(new_n235), .c(new_n230), .o(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n226), .c(new_n242), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n243), .c(new_n187), .d(new_n193), .o1(new_n246));
  xorb03aa1n02x5               g151(.a(new_n246), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n248), .c(new_n246), .d(new_n249), .o1(new_n251));
  nand02aa1n03x5               g156(.a(new_n246), .b(new_n249), .o1(new_n252));
  nona22aa1n02x4               g157(.a(new_n252), .b(new_n250), .c(new_n248), .out0(new_n253));
  nanp02aa1n03x5               g158(.a(new_n253), .b(new_n251), .o1(\s[24] ));
  norb02aa1n02x7               g159(.a(new_n249), .b(new_n250), .out0(new_n255));
  nano22aa1n03x7               g160(.a(new_n228), .b(new_n255), .c(new_n242), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n192), .c(new_n144), .d(new_n186), .o1(new_n257));
  nano22aa1n03x7               g162(.a(new_n214), .b(new_n209), .c(new_n215), .out0(new_n258));
  tech160nm_fioai012aa1n03p5x5 g163(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .o1(new_n259));
  oab012aa1n03x5               g164(.a(new_n259), .b(new_n194), .c(new_n202), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n242), .b(new_n225), .c(new_n260), .d(new_n258), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n244), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n255), .o1(new_n263));
  oai022aa1n02x5               g168(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n264));
  aob012aa1n02x5               g169(.a(new_n264), .b(\b[23] ), .c(\a[24] ), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n262), .o1(new_n266));
  nanb02aa1n03x5               g171(.a(new_n266), .b(new_n257), .out0(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n270), .b(new_n266), .c(new_n199), .d(new_n256), .o1(new_n274));
  nona22aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .out0(new_n275));
  nanp02aa1n03x5               g180(.a(new_n273), .b(new_n275), .o1(\s[26] ));
  and002aa1n12x5               g181(.a(new_n271), .b(new_n270), .o(new_n277));
  nano32aa1n03x7               g182(.a(new_n228), .b(new_n277), .c(new_n242), .d(new_n255), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n192), .c(new_n144), .d(new_n186), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[25] ), .b(\a[26] ), .o1(new_n280));
  oai022aa1n02x5               g185(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n281));
  aoi022aa1n06x5               g186(.a(new_n266), .b(new_n277), .c(new_n280), .d(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  xnbna2aa1n03x5               g188(.a(new_n283), .b(new_n282), .c(new_n279), .out0(\s[27] ));
  nand42aa1n03x5               g189(.a(new_n282), .b(new_n279), .o1(new_n285));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  xnrc02aa1n12x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n285), .d(new_n283), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n255), .b(new_n244), .c(new_n226), .d(new_n242), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n277), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n281), .b(new_n280), .o1(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n289), .d(new_n265), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n283), .b(new_n292), .c(new_n199), .d(new_n278), .o1(new_n293));
  nona22aa1n02x5               g198(.a(new_n293), .b(new_n287), .c(new_n286), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n288), .b(new_n294), .o1(\s[28] ));
  norb02aa1n03x5               g200(.a(new_n283), .b(new_n287), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n199), .d(new_n278), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n296), .o1(new_n298));
  orn002aa1n02x5               g203(.a(\a[27] ), .b(\b[26] ), .o(new_n299));
  oao003aa1n03x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n300));
  aoai13aa1n02x7               g205(.a(new_n300), .b(new_n298), .c(new_n282), .d(new_n279), .o1(new_n301));
  tech160nm_fixorc02aa1n02p5x5 g206(.a(\a[29] ), .b(\b[28] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n300), .b(new_n302), .out0(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n297), .d(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g209(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g210(.a(new_n287), .b(new_n283), .c(new_n302), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n292), .c(new_n199), .d(new_n278), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  oaoi03aa1n02x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n309), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n308), .c(new_n282), .d(new_n279), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  and002aa1n02x5               g217(.a(\b[28] ), .b(\a[29] ), .o(new_n313));
  oabi12aa1n02x5               g218(.a(new_n312), .b(\a[29] ), .c(\b[28] ), .out0(new_n314));
  oab012aa1n02x4               g219(.a(new_n314), .b(new_n300), .c(new_n313), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n311), .b(new_n312), .c(new_n307), .d(new_n315), .o1(\s[30] ));
  nand03aa1n02x5               g221(.a(new_n296), .b(new_n302), .c(new_n312), .o1(new_n317));
  nanb02aa1n03x5               g222(.a(new_n317), .b(new_n285), .out0(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(new_n321));
  aoai13aa1n02x7               g226(.a(new_n320), .b(new_n317), .c(new_n282), .d(new_n279), .o1(new_n322));
  aoi022aa1n03x5               g227(.a(new_n318), .b(new_n321), .c(new_n322), .d(new_n319), .o1(\s[31] ));
  xnrb03aa1n02x5               g228(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norb02aa1n02x5               g229(.a(new_n105), .b(new_n104), .out0(new_n325));
  aoi112aa1n02x5               g230(.a(new_n106), .b(new_n325), .c(new_n138), .d(new_n107), .o1(new_n326));
  aoib12aa1n02x5               g231(.a(new_n326), .b(new_n110), .c(new_n104), .out0(\s[4] ));
  nanp02aa1n02x5               g232(.a(new_n139), .b(new_n138), .o1(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n120), .b(new_n328), .c(new_n109), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g234(.a(\a[5] ), .b(\b[4] ), .c(new_n140), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g236(.a(new_n116), .b(new_n118), .c(new_n330), .d(new_n117), .o1(new_n332));
  aoi112aa1n02x5               g237(.a(new_n118), .b(new_n116), .c(new_n330), .d(new_n117), .o1(new_n333));
  norb02aa1n02x5               g238(.a(new_n332), .b(new_n333), .out0(\s[7] ));
  xobna2aa1n03x5               g239(.a(new_n113), .b(new_n332), .c(new_n124), .out0(\s[8] ));
  aoi112aa1n02x5               g240(.a(new_n127), .b(new_n128), .c(new_n121), .d(new_n110), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n336), .b(new_n144), .c(new_n128), .o1(\s[9] ));
endmodule


