// Benchmark "adder" written by ABC on Wed Jul 17 14:39:15 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n175, new_n176, new_n177, new_n178, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n314, new_n316, new_n317, new_n318, new_n319, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1n03x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor042aa1d18x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  tech160nm_fioai012aa1n03p5x5 g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1d18x5               g006(.a(new_n97), .b(new_n101), .c(new_n99), .d(new_n98), .out0(new_n102));
  inv000aa1d42x5               g007(.a(\a[6] ), .o1(new_n103));
  oai022aa1n03x5               g008(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n104));
  oaib12aa1n09x5               g009(.a(new_n104), .b(new_n103), .c(\b[5] ), .out0(new_n105));
  oaih12aa1n12x5               g010(.a(new_n100), .b(new_n102), .c(new_n105), .o1(new_n106));
  nor002aa1n03x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n04x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nand02aa1n03x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  aoi012aa1n06x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  tech160nm_fixnrc02aa1n02p5x5 g015(.a(\b[3] ), .b(\a[4] ), .out0(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanb02aa1n06x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  inv000aa1d42x5               g019(.a(\a[3] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(\b[2] ), .b(new_n115), .out0(new_n116));
  oao003aa1n02x5               g021(.a(\a[4] ), .b(\b[3] ), .c(new_n116), .carry(new_n117));
  oai013aa1n06x5               g022(.a(new_n117), .b(new_n111), .c(new_n110), .d(new_n114), .o1(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .out0(new_n119));
  tech160nm_fixnrc02aa1n02p5x5 g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  nor043aa1n06x5               g025(.a(new_n102), .b(new_n119), .c(new_n120), .o1(new_n121));
  aoi012aa1n12x5               g026(.a(new_n106), .b(new_n118), .c(new_n121), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor022aa1n08x5               g029(.a(\b[10] ), .b(\a[11] ), .o1(new_n125));
  nand42aa1n04x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  inv000aa1n02x5               g032(.a(new_n127), .o1(new_n128));
  nor022aa1n06x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nor022aa1n08x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand02aa1n03x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  oaih12aa1n12x5               g036(.a(new_n131), .b(new_n130), .c(new_n129), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nona23aa1n09x5               g038(.a(new_n131), .b(new_n133), .c(new_n129), .d(new_n130), .out0(new_n134));
  oaoi13aa1n02x5               g039(.a(new_n128), .b(new_n132), .c(new_n122), .d(new_n134), .o1(new_n135));
  oai112aa1n02x5               g040(.a(new_n132), .b(new_n128), .c(new_n122), .d(new_n134), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(\s[11] ));
  norp02aa1n02x5               g042(.a(new_n135), .b(new_n125), .o1(new_n138));
  xnrb03aa1n03x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1d04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1n09x5               g046(.a(new_n141), .b(new_n126), .c(new_n125), .d(new_n140), .out0(new_n142));
  norp02aa1n02x5               g047(.a(new_n142), .b(new_n134), .o1(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n106), .c(new_n118), .d(new_n121), .o1(new_n144));
  ao0012aa1n03x7               g049(.a(new_n140), .b(new_n125), .c(new_n141), .o(new_n145));
  oabi12aa1n18x5               g050(.a(new_n145), .b(new_n142), .c(new_n132), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  xnrc02aa1n12x5               g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n144), .c(new_n147), .out0(\s[13] ));
  orn002aa1n03x5               g055(.a(\a[13] ), .b(\b[12] ), .o(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n148), .c(new_n144), .d(new_n147), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .out0(new_n154));
  nor042aa1n09x5               g059(.a(new_n154), .b(new_n148), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  oaoi03aa1n03x5               g061(.a(\a[14] ), .b(\b[13] ), .c(new_n151), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n156), .c(new_n144), .d(new_n147), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g065(.a(\a[16] ), .o1(new_n161));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  xnrc02aa1n12x5               g067(.a(\b[14] ), .b(\a[15] ), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoi012aa1n03x5               g069(.a(new_n162), .b(new_n159), .c(new_n164), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[15] ), .c(new_n161), .out0(\s[16] ));
  xnrc02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .out0(new_n167));
  nor042aa1n09x5               g072(.a(new_n167), .b(new_n163), .o1(new_n168));
  nona23aa1n09x5               g073(.a(new_n155), .b(new_n168), .c(new_n142), .d(new_n134), .out0(new_n169));
  aoai13aa1n04x5               g074(.a(new_n168), .b(new_n157), .c(new_n146), .d(new_n155), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n171));
  aoib12aa1n02x5               g076(.a(new_n171), .b(new_n161), .c(\b[15] ), .out0(new_n172));
  oai112aa1n06x5               g077(.a(new_n170), .b(new_n172), .c(new_n122), .d(new_n169), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g079(.a(\a[18] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\a[17] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[16] ), .o1(new_n177));
  oaoi03aa1n03x5               g082(.a(new_n176), .b(new_n177), .c(new_n173), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[17] ), .c(new_n175), .out0(\s[18] ));
  inv000aa1d42x5               g084(.a(new_n106), .o1(new_n180));
  nand02aa1n02x5               g085(.a(new_n118), .b(new_n121), .o1(new_n181));
  aoi012aa1n09x5               g086(.a(new_n169), .b(new_n181), .c(new_n180), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n168), .o1(new_n183));
  inv040aa1n06x5               g088(.a(new_n132), .o1(new_n184));
  nano23aa1n02x4               g089(.a(new_n125), .b(new_n140), .c(new_n141), .d(new_n126), .out0(new_n185));
  aoai13aa1n04x5               g090(.a(new_n155), .b(new_n145), .c(new_n185), .d(new_n184), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n172), .b(new_n183), .c(new_n186), .d(new_n158), .o1(new_n187));
  xroi22aa1d04x5               g092(.a(new_n176), .b(\b[16] ), .c(new_n175), .d(\b[17] ), .out0(new_n188));
  tech160nm_fioai012aa1n05x5   g093(.a(new_n188), .b(new_n187), .c(new_n182), .o1(new_n189));
  oaih22aa1n04x5               g094(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n190));
  oaib12aa1n06x5               g095(.a(new_n190), .b(new_n175), .c(\b[17] ), .out0(new_n191));
  nor002aa1n16x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand02aa1n04x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n191), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g102(.a(new_n192), .o1(new_n198));
  tech160nm_fiaoi012aa1n05x5   g103(.a(new_n194), .b(new_n189), .c(new_n191), .o1(new_n199));
  nor042aa1n04x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand22aa1n06x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  nano22aa1n03x7               g107(.a(new_n199), .b(new_n198), .c(new_n202), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n177), .b(new_n176), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n195), .b(new_n205), .c(new_n173), .d(new_n188), .o1(new_n206));
  aoi012aa1n03x5               g111(.a(new_n202), .b(new_n206), .c(new_n198), .o1(new_n207));
  nor002aa1n02x5               g112(.a(new_n207), .b(new_n203), .o1(\s[20] ));
  nano23aa1n09x5               g113(.a(new_n192), .b(new_n200), .c(new_n201), .d(new_n193), .out0(new_n209));
  nand22aa1n03x5               g114(.a(new_n188), .b(new_n209), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  oaih12aa1n02x5               g116(.a(new_n211), .b(new_n187), .c(new_n182), .o1(new_n212));
  nona23aa1d18x5               g117(.a(new_n201), .b(new_n193), .c(new_n192), .d(new_n200), .out0(new_n213));
  aoi012aa1d18x5               g118(.a(new_n200), .b(new_n192), .c(new_n201), .o1(new_n214));
  oai012aa1d24x5               g119(.a(new_n214), .b(new_n213), .c(new_n191), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n212), .c(new_n216), .out0(\s[21] ));
  inv040aa1n08x5               g125(.a(new_n217), .o1(new_n221));
  aobi12aa1n03x5               g126(.a(new_n219), .b(new_n212), .c(new_n216), .out0(new_n222));
  xnrc02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  nano22aa1n03x5               g128(.a(new_n222), .b(new_n221), .c(new_n223), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n219), .b(new_n215), .c(new_n173), .d(new_n211), .o1(new_n225));
  aoi012aa1n03x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .o1(new_n226));
  norp02aa1n03x5               g131(.a(new_n226), .b(new_n224), .o1(\s[22] ));
  nano22aa1n03x7               g132(.a(new_n223), .b(new_n221), .c(new_n218), .out0(new_n228));
  and003aa1n02x5               g133(.a(new_n188), .b(new_n228), .c(new_n209), .o(new_n229));
  tech160nm_fioai012aa1n05x5   g134(.a(new_n229), .b(new_n187), .c(new_n182), .o1(new_n230));
  oao003aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .carry(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoi012aa1n06x5               g137(.a(new_n232), .b(new_n215), .c(new_n228), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[22] ), .b(\a[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n230), .c(new_n233), .out0(\s[23] ));
  nor042aa1n03x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  tech160nm_fiaoi012aa1n04x5   g143(.a(new_n234), .b(new_n230), .c(new_n233), .o1(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .out0(new_n240));
  nano22aa1n03x7               g145(.a(new_n239), .b(new_n238), .c(new_n240), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n233), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n235), .b(new_n242), .c(new_n173), .d(new_n229), .o1(new_n243));
  aoi012aa1n03x5               g148(.a(new_n240), .b(new_n243), .c(new_n238), .o1(new_n244));
  nor002aa1n02x5               g149(.a(new_n244), .b(new_n241), .o1(\s[24] ));
  nor002aa1n02x5               g150(.a(new_n240), .b(new_n234), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n210), .b(new_n228), .c(new_n246), .out0(new_n247));
  tech160nm_fioai012aa1n05x5   g152(.a(new_n247), .b(new_n187), .c(new_n182), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n214), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n228), .b(new_n249), .c(new_n209), .d(new_n205), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n246), .o1(new_n251));
  oao003aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n238), .carry(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n250), .d(new_n231), .o1(new_n253));
  xnrc02aa1n12x5               g158(.a(\b[24] ), .b(\a[25] ), .out0(new_n254));
  aoib12aa1n06x5               g159(.a(new_n254), .b(new_n248), .c(new_n253), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n254), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n256), .b(new_n253), .c(new_n173), .d(new_n247), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n255), .b(new_n257), .o1(\s[25] ));
  nor042aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xnrc02aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .out0(new_n261));
  nano22aa1n03x7               g166(.a(new_n255), .b(new_n260), .c(new_n261), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n256), .b(new_n253), .c(new_n173), .d(new_n247), .o1(new_n263));
  aoi012aa1n03x5               g168(.a(new_n261), .b(new_n263), .c(new_n260), .o1(new_n264));
  nor002aa1n02x5               g169(.a(new_n264), .b(new_n262), .o1(\s[26] ));
  nor042aa1n03x5               g170(.a(new_n261), .b(new_n254), .o1(new_n266));
  nano32aa1n03x7               g171(.a(new_n210), .b(new_n266), .c(new_n228), .d(new_n246), .out0(new_n267));
  oai012aa1n06x5               g172(.a(new_n267), .b(new_n187), .c(new_n182), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n253), .c(new_n266), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n270), .out0(\s[27] ));
  nor042aa1n03x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aobi12aa1n03x5               g179(.a(new_n271), .b(new_n268), .c(new_n270), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n03x5               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n246), .b(new_n232), .c(new_n215), .d(new_n228), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n266), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n269), .b(new_n279), .c(new_n278), .d(new_n252), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n271), .b(new_n280), .c(new_n173), .d(new_n267), .o1(new_n281));
  aoi012aa1n02x7               g186(.a(new_n276), .b(new_n281), .c(new_n274), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n271), .b(new_n276), .out0(new_n284));
  aobi12aa1n03x5               g189(.a(new_n284), .b(new_n268), .c(new_n270), .out0(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  nano22aa1n03x5               g192(.a(new_n285), .b(new_n286), .c(new_n287), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n284), .b(new_n280), .c(new_n173), .d(new_n267), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n287), .b(new_n289), .c(new_n286), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n271), .b(new_n287), .c(new_n276), .out0(new_n293));
  aobi12aa1n03x5               g198(.a(new_n293), .b(new_n268), .c(new_n270), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n293), .b(new_n280), .c(new_n173), .d(new_n267), .o1(new_n298));
  aoi012aa1n02x7               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n03x5               g204(.a(new_n299), .b(new_n297), .o1(\s[30] ));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  norb02aa1n02x5               g206(.a(new_n293), .b(new_n296), .out0(new_n302));
  aobi12aa1n03x5               g207(.a(new_n302), .b(new_n268), .c(new_n270), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n304));
  nano22aa1n03x5               g209(.a(new_n303), .b(new_n301), .c(new_n304), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n302), .b(new_n280), .c(new_n173), .d(new_n267), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n301), .b(new_n306), .c(new_n304), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  xnbna2aa1n03x5               g213(.a(new_n110), .b(new_n113), .c(new_n116), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g217(.a(\b[4] ), .b(\a[5] ), .o1(new_n313));
  aoib12aa1n02x5               g218(.a(new_n313), .b(new_n118), .c(new_n120), .out0(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(new_n103), .out0(\s[6] ));
  and002aa1n02x5               g220(.a(\b[5] ), .b(\a[6] ), .o(new_n316));
  nanb02aa1n02x5               g221(.a(new_n119), .b(new_n314), .out0(new_n317));
  nona23aa1n02x4               g222(.a(new_n317), .b(new_n101), .c(new_n99), .d(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n99), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n316), .b(new_n317), .c(new_n319), .d(new_n101), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(\s[7] ));
  norb02aa1n02x5               g226(.a(new_n97), .b(new_n98), .out0(new_n322));
  xnbna2aa1n03x5               g227(.a(new_n322), .b(new_n318), .c(new_n319), .out0(\s[8] ));
  xnrb03aa1n02x5               g228(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


