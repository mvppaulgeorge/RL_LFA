// Benchmark "adder" written by ABC on Thu Jul 18 08:50:54 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n310,
    new_n313, new_n314, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand42aa1n06x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  nor022aa1n16x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n03x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanp02aa1n03x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[3] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[2] ), .o1(new_n110));
  aoai13aa1n06x5               g015(.a(new_n104), .b(new_n103), .c(new_n109), .d(new_n110), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n06x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nona22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oao003aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n123));
  tech160nm_fioai012aa1n03p5x5 g028(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n124));
  aobi12aa1n06x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .out0(new_n125));
  aoai13aa1n04x5               g030(.a(new_n125), .b(new_n119), .c(new_n108), .d(new_n111), .o1(new_n126));
  tech160nm_fixorc02aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(\b[9] ), .o1(new_n131));
  oao003aa1n02x5               g036(.a(new_n97), .b(new_n131), .c(new_n98), .carry(new_n132));
  aoi013aa1n03x5               g037(.a(new_n132), .b(new_n126), .c(new_n127), .d(new_n130), .o1(new_n133));
  xnrb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n03x5               g039(.a(\a[11] ), .b(\b[10] ), .c(new_n133), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  tech160nm_fioaoi03aa1n03p5x5 g041(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n137));
  nona23aa1n09x5               g042(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n138));
  oaih12aa1n06x5               g043(.a(new_n111), .b(new_n138), .c(new_n137), .o1(new_n139));
  nona23aa1n02x5               g044(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n140));
  nor043aa1n03x5               g045(.a(new_n140), .b(new_n117), .c(new_n118), .o1(new_n141));
  aob012aa1n12x5               g046(.a(new_n124), .b(new_n116), .c(new_n123), .out0(new_n142));
  nor022aa1n08x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanp02aa1n09x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nor022aa1n08x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1n08x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nona23aa1n09x5               g051(.a(new_n146), .b(new_n144), .c(new_n143), .d(new_n145), .out0(new_n147));
  nano22aa1n02x4               g052(.a(new_n147), .b(new_n130), .c(new_n127), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n142), .c(new_n139), .d(new_n141), .o1(new_n149));
  nano23aa1n06x5               g054(.a(new_n143), .b(new_n145), .c(new_n146), .d(new_n144), .out0(new_n150));
  oaih12aa1n02x5               g055(.a(new_n146), .b(new_n145), .c(new_n143), .o1(new_n151));
  aobi12aa1n03x7               g056(.a(new_n151), .b(new_n150), .c(new_n132), .out0(new_n152));
  xorc02aa1n12x5               g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n149), .c(new_n152), .out0(\s[13] ));
  nor042aa1n03x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  inv000aa1n02x5               g060(.a(new_n155), .o1(new_n156));
  oaoi03aa1n02x5               g061(.a(new_n97), .b(new_n131), .c(new_n98), .o1(new_n157));
  tech160nm_fioai012aa1n04x5   g062(.a(new_n151), .b(new_n147), .c(new_n157), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n153), .b(new_n158), .c(new_n126), .d(new_n148), .o1(new_n159));
  xorc02aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n159), .c(new_n156), .out0(\s[14] ));
  nanp02aa1n02x5               g066(.a(new_n160), .b(new_n153), .o1(new_n162));
  oao003aa1n06x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .carry(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n162), .c(new_n149), .d(new_n152), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1d28x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1d28x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n166), .b(new_n170), .c(new_n164), .d(new_n167), .o1(new_n172));
  norb02aa1n03x4               g077(.a(new_n171), .b(new_n172), .out0(\s[16] ));
  nano23aa1d15x5               g078(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n174));
  nand23aa1d12x5               g079(.a(new_n174), .b(new_n153), .c(new_n160), .o1(new_n175));
  nano32aa1n09x5               g080(.a(new_n175), .b(new_n150), .c(new_n127), .d(new_n130), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n142), .c(new_n139), .d(new_n141), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n169), .b(new_n168), .c(new_n166), .o1(new_n178));
  oaib12aa1n09x5               g083(.a(new_n178), .b(new_n163), .c(new_n174), .out0(new_n179));
  aoib12aa1n12x5               g084(.a(new_n179), .b(new_n158), .c(new_n175), .out0(new_n180));
  nanp02aa1n06x5               g085(.a(new_n177), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nor042aa1n04x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  aoi112aa1n09x5               g094(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n190));
  nor042aa1n06x5               g095(.a(new_n190), .b(new_n189), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n09x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n06x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n197));
  norb02aa1n02x7               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand02aa1d28x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n15x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  oaoi13aa1n06x5               g108(.a(new_n203), .b(new_n196), .c(\a[19] ), .d(\b[18] ), .o1(new_n204));
  nona22aa1n02x5               g109(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n205), .b(new_n204), .out0(\s[20] ));
  nona23aa1n09x5               g111(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n207));
  inv040aa1n03x5               g112(.a(new_n207), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n188), .b(new_n208), .o1(new_n209));
  oai012aa1n06x5               g114(.a(new_n201), .b(new_n200), .c(new_n193), .o1(new_n210));
  oai012aa1n18x5               g115(.a(new_n210), .b(new_n207), .c(new_n191), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n209), .c(new_n177), .d(new_n180), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n12x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xorc02aa1n12x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n215), .c(new_n213), .d(new_n216), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n215), .b(new_n217), .c(new_n213), .d(new_n216), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n218), .b(new_n219), .out0(\s[22] ));
  nand02aa1d12x5               g125(.a(new_n217), .b(new_n216), .o1(new_n221));
  nanb03aa1n06x5               g126(.a(new_n221), .b(new_n188), .c(new_n208), .out0(new_n222));
  oai112aa1n06x5               g127(.a(new_n195), .b(new_n202), .c(new_n190), .d(new_n189), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv040aa1d32x5               g129(.a(\b[21] ), .o1(new_n225));
  oaoi03aa1n12x5               g130(.a(new_n224), .b(new_n225), .c(new_n215), .o1(new_n226));
  aoai13aa1n12x5               g131(.a(new_n226), .b(new_n221), .c(new_n223), .d(new_n210), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n222), .c(new_n177), .d(new_n180), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  tech160nm_fixorc02aa1n02p5x5 g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n234), .b(new_n235), .out0(\s[24] ));
  and002aa1n06x5               g141(.a(new_n233), .b(new_n232), .o(new_n237));
  nona23aa1n02x4               g142(.a(new_n237), .b(new_n188), .c(new_n221), .d(new_n207), .out0(new_n238));
  inv000aa1d42x5               g143(.a(\a[24] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\b[23] ), .o1(new_n240));
  oao003aa1n02x5               g145(.a(new_n239), .b(new_n240), .c(new_n231), .carry(new_n241));
  tech160nm_fiaoi012aa1n05x5   g146(.a(new_n241), .b(new_n227), .c(new_n237), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n238), .c(new_n177), .d(new_n180), .o1(new_n243));
  xorb03aa1n03x5               g148(.a(new_n243), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  xorc02aa1n12x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  tech160nm_fixorc02aa1n05x5   g151(.a(\a[26] ), .b(\b[25] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n248));
  aoi112aa1n03x5               g153(.a(new_n245), .b(new_n247), .c(new_n243), .d(new_n246), .o1(new_n249));
  norb02aa1n03x4               g154(.a(new_n248), .b(new_n249), .out0(\s[26] ));
  oai022aa1n02x5               g155(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n251));
  aboi22aa1n03x5               g156(.a(new_n163), .b(new_n174), .c(new_n251), .d(new_n169), .out0(new_n252));
  oai012aa1n02x7               g157(.a(new_n252), .b(new_n152), .c(new_n175), .o1(new_n253));
  and002aa1n12x5               g158(.a(new_n247), .b(new_n246), .o(new_n254));
  nano22aa1n06x5               g159(.a(new_n222), .b(new_n237), .c(new_n254), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n253), .c(new_n126), .d(new_n176), .o1(new_n256));
  aoai13aa1n09x5               g161(.a(new_n254), .b(new_n241), .c(new_n227), .d(new_n237), .o1(new_n257));
  oai022aa1n02x5               g162(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n258));
  aob012aa1n02x5               g163(.a(new_n258), .b(\b[25] ), .c(\a[26] ), .out0(new_n259));
  xorc02aa1n12x5               g164(.a(\a[27] ), .b(\b[26] ), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoi013aa1n03x5               g166(.a(new_n261), .b(new_n256), .c(new_n257), .d(new_n259), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n255), .o1(new_n263));
  aoi012aa1n06x5               g168(.a(new_n263), .b(new_n177), .c(new_n180), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n221), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n226), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n237), .b(new_n266), .c(new_n211), .d(new_n265), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n241), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n254), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n259), .b(new_n269), .c(new_n267), .d(new_n268), .o1(new_n270));
  norp03aa1n02x5               g175(.a(new_n270), .b(new_n264), .c(new_n260), .o1(new_n271));
  nor002aa1n02x5               g176(.a(new_n262), .b(new_n271), .o1(\s[27] ));
  nor042aa1n03x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  oai012aa1n02x7               g179(.a(new_n260), .b(new_n270), .c(new_n264), .o1(new_n275));
  xnrc02aa1n12x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  aoi012aa1n02x7               g181(.a(new_n276), .b(new_n275), .c(new_n274), .o1(new_n277));
  nano22aa1n03x5               g182(.a(new_n262), .b(new_n274), .c(new_n276), .out0(new_n278));
  norp02aa1n03x5               g183(.a(new_n277), .b(new_n278), .o1(\s[28] ));
  norb02aa1d21x5               g184(.a(new_n260), .b(new_n276), .out0(new_n280));
  oai012aa1n03x5               g185(.a(new_n280), .b(new_n270), .c(new_n264), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  aoi012aa1n03x5               g188(.a(new_n283), .b(new_n281), .c(new_n282), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n280), .o1(new_n285));
  aoi013aa1n02x5               g190(.a(new_n285), .b(new_n256), .c(new_n257), .d(new_n259), .o1(new_n286));
  nano22aa1n03x5               g191(.a(new_n286), .b(new_n282), .c(new_n283), .out0(new_n287));
  norp02aa1n03x5               g192(.a(new_n284), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g194(.a(new_n260), .b(new_n283), .c(new_n276), .out0(new_n290));
  oai012aa1n03x5               g195(.a(new_n290), .b(new_n270), .c(new_n264), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n290), .o1(new_n295));
  aoi013aa1n02x5               g200(.a(new_n295), .b(new_n256), .c(new_n257), .d(new_n259), .o1(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n293), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n294), .b(new_n297), .o1(\s[30] ));
  norb02aa1n02x7               g203(.a(new_n290), .b(new_n293), .out0(new_n299));
  inv020aa1n02x5               g204(.a(new_n299), .o1(new_n300));
  aoi013aa1n02x5               g205(.a(new_n300), .b(new_n256), .c(new_n257), .d(new_n259), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n03x5               g209(.a(new_n299), .b(new_n270), .c(new_n264), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xorb03aa1n02x5               g212(.a(new_n137), .b(\b[2] ), .c(new_n109), .out0(\s[3] ));
  norb02aa1n02x5               g213(.a(new_n104), .b(new_n103), .out0(new_n309));
  aoi112aa1n02x5               g214(.a(new_n105), .b(new_n309), .c(new_n102), .d(new_n106), .o1(new_n310));
  aoib12aa1n02x5               g215(.a(new_n310), .b(new_n139), .c(new_n103), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n139), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g217(.a(\a[5] ), .b(\b[4] ), .o(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n118), .c(new_n108), .d(new_n111), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n03x5               g220(.a(new_n120), .b(new_n121), .c(new_n314), .carry(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n114), .b(new_n316), .c(new_n115), .o1(new_n318));
  xnrb03aa1n03x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


