// Benchmark "adder" written by ABC on Thu Jul 18 09:12:20 2024

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
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n290, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n308, new_n310,
    new_n313, new_n314, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  tech160nm_fixnrc02aa1n02p5x5 g004(.a(\b[7] ), .b(\a[8] ), .out0(new_n100));
  xnrc02aa1n06x5               g005(.a(\b[6] ), .b(\a[7] ), .out0(new_n101));
  norp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  tech160nm_fixnrc02aa1n05x5   g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp02aa1n02x5               g009(.a(new_n104), .b(new_n103), .o1(new_n105));
  nor042aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  aoi022aa1d24x5               g011(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n107));
  nor042aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norb02aa1n06x4               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nor042aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  norb02aa1n03x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  oai112aa1n06x5               g018(.a(new_n110), .b(new_n113), .c(new_n107), .d(new_n106), .o1(new_n114));
  tech160nm_fiaoi012aa1n03p5x5 g019(.a(new_n108), .b(new_n111), .c(new_n109), .o1(new_n115));
  nanp02aa1n06x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  nand23aa1n06x5               g021(.a(new_n116), .b(new_n102), .c(new_n105), .o1(new_n117));
  orn002aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .o(new_n118));
  norp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  aob012aa1n02x5               g024(.a(new_n119), .b(\b[7] ), .c(\a[8] ), .out0(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aob012aa1n06x5               g026(.a(new_n121), .b(\b[5] ), .c(\a[6] ), .out0(new_n122));
  norp03aa1n02x5               g027(.a(new_n122), .b(new_n101), .c(new_n100), .o1(new_n123));
  nano22aa1n06x5               g028(.a(new_n123), .b(new_n118), .c(new_n120), .out0(new_n124));
  nanp02aa1n03x5               g029(.a(new_n117), .b(new_n124), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  nanp02aa1n03x5               g031(.a(new_n125), .b(new_n126), .o1(new_n127));
  nona22aa1n06x5               g032(.a(new_n127), .b(new_n99), .c(new_n98), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n98), .b(new_n99), .c(new_n125), .d(new_n126), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(\s[10] ));
  and002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  nor042aa1d18x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  nona22aa1d18x5               g039(.a(new_n128), .b(new_n134), .c(new_n131), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n132), .o1(new_n137));
  aboi22aa1n03x5               g042(.a(new_n131), .b(new_n128), .c(new_n137), .d(new_n133), .out0(new_n138));
  norp02aa1n02x5               g043(.a(new_n138), .b(new_n136), .o1(\s[11] ));
  nor042aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  nanp03aa1n06x5               g047(.a(new_n135), .b(new_n137), .c(new_n142), .o1(new_n143));
  tech160nm_fiaoi012aa1n03p5x5 g048(.a(new_n142), .b(new_n135), .c(new_n137), .o1(new_n144));
  norb02aa1n03x4               g049(.a(new_n143), .b(new_n144), .out0(\s[12] ));
  nona23aa1d18x5               g050(.a(new_n141), .b(new_n133), .c(new_n132), .d(new_n140), .out0(new_n146));
  nanb03aa1n02x5               g051(.a(new_n146), .b(new_n97), .c(new_n126), .out0(new_n147));
  tech160nm_fioai012aa1n03p5x5 g052(.a(new_n141), .b(new_n140), .c(new_n132), .o1(new_n148));
  oai022aa1n06x5               g053(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n149));
  aob012aa1n03x5               g054(.a(new_n149), .b(\b[9] ), .c(\a[10] ), .out0(new_n150));
  oai012aa1n18x5               g055(.a(new_n148), .b(new_n146), .c(new_n150), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n147), .c(new_n117), .d(new_n124), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n12x5               g063(.a(\a[15] ), .b(\b[14] ), .out0(new_n159));
  nor002aa1d32x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1d24x5               g066(.a(new_n161), .b(new_n156), .c(new_n155), .d(new_n160), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  oai012aa1n12x5               g068(.a(new_n161), .b(new_n160), .c(new_n155), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n159), .b(new_n165), .c(new_n153), .d(new_n163), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n159), .b(new_n165), .c(new_n153), .d(new_n163), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g073(.a(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(\b[14] ), .b(new_n169), .out0(new_n170));
  xnrc02aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  nanp03aa1n02x5               g076(.a(new_n166), .b(new_n170), .c(new_n171), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n171), .b(new_n166), .c(new_n170), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[16] ));
  norb03aa1d15x5               g079(.a(new_n159), .b(new_n162), .c(new_n171), .out0(new_n175));
  nona23aa1d18x5               g080(.a(new_n175), .b(new_n126), .c(new_n98), .d(new_n146), .out0(new_n176));
  aoi112aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n177));
  xorc02aa1n02x5               g082(.a(\a[16] ), .b(\b[15] ), .out0(new_n178));
  nanb03aa1n03x5               g083(.a(new_n164), .b(new_n178), .c(new_n159), .out0(new_n179));
  oai012aa1n06x5               g084(.a(new_n179), .b(\b[15] ), .c(\a[16] ), .o1(new_n180));
  aoi112aa1n06x5               g085(.a(new_n180), .b(new_n177), .c(new_n175), .d(new_n151), .o1(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n176), .c(new_n117), .d(new_n124), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n182), .o1(new_n186));
  xnrb03aa1n03x5               g091(.a(new_n186), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nor042aa1n03x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nand42aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nano23aa1n06x5               g096(.a(new_n188), .b(new_n190), .c(new_n191), .d(new_n189), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n191), .b(new_n190), .c(new_n184), .d(new_n185), .o1(new_n193));
  inv040aa1n02x5               g098(.a(new_n193), .o1(new_n194));
  nor042aa1n06x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n194), .c(new_n182), .d(new_n192), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n182), .d(new_n192), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand22aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n03x5               g109(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n195), .o1(new_n206));
  aobi12aa1n03x5               g111(.a(new_n204), .b(new_n198), .c(new_n206), .out0(new_n207));
  norb02aa1n03x4               g112(.a(new_n205), .b(new_n207), .out0(\s[20] ));
  nona23aa1n09x5               g113(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n209));
  norb02aa1n06x4               g114(.a(new_n192), .b(new_n209), .out0(new_n210));
  aoi012aa1d18x5               g115(.a(new_n202), .b(new_n195), .c(new_n203), .o1(new_n211));
  tech160nm_fioai012aa1n05x5   g116(.a(new_n211), .b(new_n209), .c(new_n193), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n182), .d(new_n210), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n182), .d(new_n210), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  nor042aa1n03x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  nona22aa1n03x5               g123(.a(new_n214), .b(new_n218), .c(new_n217), .out0(new_n219));
  inv040aa1n03x5               g124(.a(new_n217), .o1(new_n220));
  aobi12aa1n06x5               g125(.a(new_n218), .b(new_n214), .c(new_n220), .out0(new_n221));
  norb02aa1n03x4               g126(.a(new_n219), .b(new_n221), .out0(\s[22] ));
  nano23aa1n06x5               g127(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  and003aa1n02x5               g131(.a(new_n226), .b(new_n223), .c(new_n192), .o(new_n227));
  oao003aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .carry(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n212), .c(new_n226), .o1(new_n230));
  inv040aa1n03x5               g135(.a(new_n230), .o1(new_n231));
  tech160nm_fixorc02aa1n02p5x5 g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n182), .d(new_n227), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n232), .b(new_n231), .c(new_n182), .d(new_n227), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n233), .b(new_n234), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  nona22aa1n03x5               g142(.a(new_n233), .b(new_n237), .c(new_n236), .out0(new_n238));
  inv000aa1n02x5               g143(.a(new_n236), .o1(new_n239));
  aobi12aa1n06x5               g144(.a(new_n237), .b(new_n233), .c(new_n239), .out0(new_n240));
  norb02aa1n03x4               g145(.a(new_n238), .b(new_n240), .out0(\s[24] ));
  and002aa1n06x5               g146(.a(new_n237), .b(new_n232), .o(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n226), .c(new_n223), .d(new_n192), .out0(new_n244));
  inv040aa1n03x5               g149(.a(new_n211), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n226), .b(new_n245), .c(new_n223), .d(new_n194), .o1(new_n246));
  oao003aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .c(new_n239), .carry(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n243), .c(new_n246), .d(new_n228), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n244), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n182), .d(new_n244), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  nor042aa1n03x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n03x5               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n253), .o1(new_n256));
  aobi12aa1n06x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  nano32aa1n02x4               g163(.a(new_n147), .b(new_n178), .c(new_n159), .d(new_n163), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n175), .b(new_n151), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n180), .c(new_n177), .out0(new_n261));
  inv000aa1d42x5               g166(.a(\a[25] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\a[26] ), .o1(new_n263));
  xroi22aa1d04x5               g168(.a(new_n262), .b(\b[24] ), .c(new_n263), .d(\b[25] ), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  nano32aa1n09x5               g170(.a(new_n265), .b(new_n210), .c(new_n226), .d(new_n242), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n261), .c(new_n125), .d(new_n259), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n268));
  aobi12aa1n06x5               g173(.a(new_n268), .b(new_n248), .c(new_n264), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  inv040aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  aobi12aa1n02x5               g178(.a(new_n270), .b(new_n269), .c(new_n267), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n03x5               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n242), .b(new_n229), .c(new_n212), .d(new_n226), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n268), .b(new_n265), .c(new_n277), .d(new_n247), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n270), .b(new_n278), .c(new_n182), .d(new_n266), .o1(new_n279));
  aoi012aa1n02x7               g184(.a(new_n275), .b(new_n279), .c(new_n273), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n270), .b(new_n275), .out0(new_n282));
  aobi12aa1n02x5               g187(.a(new_n282), .b(new_n269), .c(new_n267), .out0(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n283), .b(new_n284), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n282), .b(new_n278), .c(new_n182), .d(new_n266), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n286), .o1(\s[29] ));
  nanp02aa1n02x5               g194(.a(\b[0] ), .b(\a[1] ), .o1(new_n290));
  xorb03aa1n02x5               g195(.a(new_n290), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n270), .b(new_n285), .c(new_n275), .out0(new_n292));
  aobi12aa1n02x5               g197(.a(new_n292), .b(new_n269), .c(new_n267), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n292), .b(new_n278), .c(new_n182), .d(new_n266), .o1(new_n297));
  aoi012aa1n03x5               g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  norp02aa1n03x5               g203(.a(new_n298), .b(new_n296), .o1(\s[30] ));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  norb02aa1n02x5               g205(.a(new_n292), .b(new_n295), .out0(new_n301));
  aobi12aa1n02x5               g206(.a(new_n301), .b(new_n269), .c(new_n267), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n303));
  nano22aa1n03x5               g208(.a(new_n302), .b(new_n300), .c(new_n303), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n301), .b(new_n278), .c(new_n182), .d(new_n266), .o1(new_n305));
  aoi012aa1n03x5               g210(.a(new_n300), .b(new_n305), .c(new_n303), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  oabi12aa1n02x5               g212(.a(new_n107), .b(\a[2] ), .c(\b[1] ), .out0(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi13aa1n02x5               g214(.a(new_n111), .b(new_n112), .c(new_n107), .d(new_n106), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(new_n310), .b(new_n110), .out0(\s[4] ));
  xobna2aa1n03x5               g216(.a(new_n104), .b(new_n114), .c(new_n115), .out0(\s[5] ));
  orn002aa1n02x5               g217(.a(\a[5] ), .b(\b[4] ), .o(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n104), .c(new_n114), .d(new_n115), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g220(.a(new_n122), .b(new_n116), .c(new_n105), .out0(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g222(.a(new_n119), .b(new_n316), .c(new_n101), .out0(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g224(.a(new_n126), .b(new_n117), .c(new_n124), .out0(\s[9] ));
endmodule


