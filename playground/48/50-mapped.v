// Benchmark "adder" written by ABC on Thu Jul 18 13:14:05 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n316, new_n317, new_n319, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand22aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  norp02aa1n24x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor002aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  tech160nm_fioai012aa1n05x5   g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n99), .b(new_n107), .c(new_n101), .d(new_n100), .out0(new_n108));
  oai012aa1n04x7               g013(.a(new_n102), .b(new_n108), .c(new_n106), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand22aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  xnrc02aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  tech160nm_fioai012aa1n03p5x5 g024(.a(new_n118), .b(new_n115), .c(new_n119), .o1(new_n120));
  tech160nm_fioai012aa1n03p5x5 g025(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n121));
  oai013aa1n09x5               g026(.a(new_n120), .b(new_n116), .c(new_n115), .d(new_n121), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n124));
  nor042aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  tech160nm_finand02aa1n03p5x5 g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n03x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g033(.a(new_n109), .b(new_n117), .o1(new_n129));
  nanb02aa1n02x5               g034(.a(new_n122), .b(new_n129), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n127), .b(new_n97), .c(new_n130), .d(new_n123), .o1(new_n131));
  tech160nm_fioai012aa1n04x5   g036(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n132));
  nor042aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n132), .out0(\s[11] ));
  aobi12aa1n02x5               g041(.a(new_n135), .b(new_n131), .c(new_n132), .out0(new_n137));
  nor042aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  oai012aa1n02x5               g045(.a(new_n140), .b(new_n137), .c(new_n133), .o1(new_n141));
  norb03aa1n06x5               g046(.a(new_n139), .b(new_n133), .c(new_n138), .out0(new_n142));
  oaib12aa1n02x5               g047(.a(new_n141), .b(new_n137), .c(new_n142), .out0(\s[12] ));
  nona23aa1n03x5               g048(.a(new_n139), .b(new_n134), .c(new_n133), .d(new_n138), .out0(new_n144));
  nano22aa1n02x4               g049(.a(new_n144), .b(new_n123), .c(new_n127), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n146));
  and002aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o(new_n147));
  oai022aa1n02x5               g052(.a(new_n144), .b(new_n132), .c(new_n142), .d(new_n147), .o1(new_n148));
  nanb02aa1n02x5               g053(.a(new_n148), .b(new_n146), .out0(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand22aa1n12x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1d04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1n06x5               g061(.a(new_n151), .b(new_n155), .c(new_n156), .d(new_n152), .out0(new_n157));
  aoi012aa1n02x5               g062(.a(new_n155), .b(new_n151), .c(new_n156), .o1(new_n158));
  aobi12aa1n02x7               g063(.a(new_n158), .b(new_n148), .c(new_n157), .out0(new_n159));
  oaib12aa1n02x5               g064(.a(new_n159), .b(new_n146), .c(new_n157), .out0(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nand42aa1d28x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nor042aa1n04x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n24x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nano23aa1d15x5               g072(.a(new_n162), .b(new_n166), .c(new_n167), .d(new_n163), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nano23aa1n06x5               g074(.a(new_n133), .b(new_n138), .c(new_n139), .d(new_n134), .out0(new_n170));
  nand02aa1d04x5               g075(.a(new_n168), .b(new_n157), .o1(new_n171));
  nano32aa1d12x5               g076(.a(new_n171), .b(new_n170), .c(new_n127), .d(new_n123), .out0(new_n172));
  aoai13aa1n12x5               g077(.a(new_n172), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n173));
  tech160nm_fiaoi012aa1n03p5x5 g078(.a(new_n166), .b(new_n162), .c(new_n167), .o1(new_n174));
  oai112aa1n06x5               g079(.a(new_n173), .b(new_n174), .c(new_n159), .d(new_n169), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g081(.a(\a[18] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n173), .o1(new_n178));
  inv030aa1n04x5               g083(.a(new_n132), .o1(new_n179));
  oa0012aa1n02x5               g084(.a(new_n139), .b(new_n138), .c(new_n133), .o(new_n180));
  aoai13aa1n04x5               g085(.a(new_n157), .b(new_n180), .c(new_n170), .d(new_n179), .o1(new_n181));
  aoai13aa1n12x5               g086(.a(new_n174), .b(new_n169), .c(new_n181), .d(new_n158), .o1(new_n182));
  nor042aa1n04x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  oaoi13aa1n06x5               g089(.a(new_n183), .b(new_n184), .c(new_n178), .d(new_n182), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  xroi22aa1d06x4               g092(.a(new_n187), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n188));
  nor042aa1n02x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nand22aa1n03x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  aoi012aa1n09x5               g095(.a(new_n189), .b(new_n183), .c(new_n190), .o1(new_n191));
  inv020aa1n04x5               g096(.a(new_n191), .o1(new_n192));
  oaoi13aa1n04x5               g097(.a(new_n192), .b(new_n188), .c(new_n178), .d(new_n182), .o1(new_n193));
  nor042aa1n04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand02aa1n06x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  xnrc02aa1n02x5               g101(.a(new_n193), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g103(.a(new_n182), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n188), .o1(new_n200));
  aoai13aa1n04x5               g105(.a(new_n191), .b(new_n200), .c(new_n199), .d(new_n173), .o1(new_n201));
  nor042aa1n04x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand02aa1n08x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n194), .c(new_n201), .d(new_n196), .o1(new_n205));
  norb03aa1n02x5               g110(.a(new_n203), .b(new_n194), .c(new_n202), .out0(new_n206));
  oaib12aa1n02x5               g111(.a(new_n206), .b(new_n193), .c(new_n196), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n205), .b(new_n207), .o1(\s[20] ));
  nano23aa1n09x5               g113(.a(new_n194), .b(new_n202), .c(new_n203), .d(new_n195), .out0(new_n209));
  nand22aa1n12x5               g114(.a(new_n188), .b(new_n209), .o1(new_n210));
  nona23aa1d18x5               g115(.a(new_n203), .b(new_n195), .c(new_n194), .d(new_n202), .out0(new_n211));
  oai012aa1n12x5               g116(.a(new_n203), .b(new_n202), .c(new_n194), .o1(new_n212));
  oai012aa1d24x5               g117(.a(new_n212), .b(new_n211), .c(new_n191), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n210), .c(new_n199), .d(new_n173), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n12x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n210), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n218), .b(new_n213), .c(new_n175), .d(new_n222), .o1(new_n223));
  nona22aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n217), .out0(new_n224));
  nanp02aa1n03x5               g129(.a(new_n221), .b(new_n224), .o1(\s[22] ));
  nano32aa1n03x7               g130(.a(new_n200), .b(new_n219), .c(new_n209), .d(new_n218), .out0(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\a[21] ), .o1(new_n228));
  inv040aa1d32x5               g133(.a(\a[22] ), .o1(new_n229));
  xroi22aa1d06x4               g134(.a(new_n228), .b(\b[20] ), .c(new_n229), .d(\b[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n229), .b(new_n231), .c(new_n217), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1d18x5               g138(.a(new_n233), .b(new_n213), .c(new_n230), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n227), .c(new_n199), .d(new_n173), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n06x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xnrc02aa1n03x5               g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n234), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n238), .b(new_n241), .c(new_n175), .d(new_n226), .o1(new_n242));
  nona22aa1n03x5               g147(.a(new_n242), .b(new_n239), .c(new_n237), .out0(new_n243));
  nanp02aa1n02x5               g148(.a(new_n240), .b(new_n243), .o1(\s[24] ));
  norb02aa1n02x5               g149(.a(new_n238), .b(new_n239), .out0(new_n245));
  nano22aa1n03x7               g150(.a(new_n210), .b(new_n245), .c(new_n230), .out0(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n182), .c(new_n130), .d(new_n172), .o1(new_n247));
  inv020aa1n04x5               g152(.a(new_n212), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n230), .b(new_n248), .c(new_n209), .d(new_n192), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n245), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n251));
  oab012aa1n02x4               g156(.a(new_n251), .b(\a[24] ), .c(\b[23] ), .out0(new_n252));
  aoai13aa1n12x5               g157(.a(new_n252), .b(new_n250), .c(new_n249), .d(new_n232), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n247), .c(new_n254), .out0(\s[25] ));
  nanp02aa1n02x5               g161(.a(new_n247), .b(new_n254), .o1(new_n257));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n258), .c(new_n257), .d(new_n255), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n255), .b(new_n253), .c(new_n175), .d(new_n246), .o1(new_n262));
  nona22aa1n03x5               g167(.a(new_n262), .b(new_n260), .c(new_n258), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n261), .b(new_n263), .o1(\s[26] ));
  and002aa1n02x5               g169(.a(new_n259), .b(new_n255), .o(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  nano23aa1d15x5               g171(.a(new_n210), .b(new_n266), .c(new_n245), .d(new_n230), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n182), .c(new_n130), .d(new_n172), .o1(new_n268));
  orn002aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .o(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n270));
  aobi12aa1n18x5               g175(.a(new_n270), .b(new_n253), .c(new_n265), .out0(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n245), .b(new_n233), .c(new_n213), .d(new_n230), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n270), .b(new_n266), .c(new_n276), .d(new_n252), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n272), .b(new_n277), .c(new_n175), .d(new_n267), .o1(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  tech160nm_fiaoi012aa1n02p5x5 g184(.a(new_n279), .b(new_n278), .c(new_n275), .o1(new_n280));
  aobi12aa1n02x7               g185(.a(new_n272), .b(new_n268), .c(new_n271), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n275), .c(new_n279), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[28] ));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  norb02aa1n02x5               g189(.a(new_n272), .b(new_n279), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n277), .c(new_n175), .d(new_n267), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n287));
  aoi012aa1n02x7               g192(.a(new_n284), .b(new_n286), .c(new_n287), .o1(new_n288));
  aobi12aa1n02x7               g193(.a(new_n285), .b(new_n268), .c(new_n271), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n284), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  norb03aa1n02x5               g198(.a(new_n272), .b(new_n284), .c(new_n279), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n277), .c(new_n175), .d(new_n267), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  aoi012aa1n02x7               g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n294), .b(new_n268), .c(new_n271), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n294), .b(new_n293), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n277), .c(new_n175), .d(new_n267), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n301), .b(new_n268), .c(new_n271), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g217(.a(new_n112), .b(new_n109), .c(new_n113), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g219(.a(new_n110), .b(new_n111), .out0(new_n315));
  oaoi13aa1n02x5               g220(.a(new_n116), .b(new_n121), .c(new_n313), .d(new_n315), .o1(new_n316));
  oai112aa1n02x5               g221(.a(new_n121), .b(new_n116), .c(new_n313), .d(new_n315), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(\s[7] ));
  oabi12aa1n02x5               g223(.a(new_n115), .b(\a[7] ), .c(\b[6] ), .out0(new_n319));
  oai012aa1n02x5               g224(.a(new_n115), .b(new_n316), .c(new_n119), .o1(new_n320));
  oai012aa1n02x5               g225(.a(new_n320), .b(new_n316), .c(new_n319), .o1(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n130), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


