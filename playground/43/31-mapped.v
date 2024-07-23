// Benchmark "adder" written by ABC on Thu Jul 18 10:16:18 2024

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
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n311, new_n313,
    new_n314, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nand42aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor022aa1n16x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nanp02aa1n06x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nona23aa1n12x5               g009(.a(new_n101), .b(new_n104), .c(new_n103), .d(new_n102), .out0(new_n105));
  inv000aa1d42x5               g010(.a(\a[6] ), .o1(new_n106));
  oaih22aa1n06x5               g011(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n107));
  oaib12aa1n18x5               g012(.a(new_n107), .b(new_n106), .c(\b[5] ), .out0(new_n108));
  inv040aa1n06x5               g013(.a(new_n103), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(\a[8] ), .b(\b[7] ), .c(new_n109), .o1(new_n110));
  oabi12aa1n06x5               g015(.a(new_n110), .b(new_n105), .c(new_n108), .out0(new_n111));
  nor002aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nand22aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  aoi012aa1n06x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  norp02aa1n09x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nanp02aa1n04x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nor022aa1n16x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand42aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nona23aa1n09x5               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  tech160nm_fiaoi012aa1n03p5x5 g025(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n121));
  oai012aa1n12x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  tech160nm_fixnrc02aa1n02p5x5 g027(.a(\b[5] ), .b(\a[6] ), .out0(new_n123));
  tech160nm_fixnrc02aa1n04x5   g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nor043aa1n06x5               g029(.a(new_n105), .b(new_n123), .c(new_n124), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n111), .c(new_n122), .d(new_n125), .o1(new_n127));
  obai22aa1n02x7               g032(.a(new_n127), .b(new_n100), .c(new_n97), .d(new_n99), .out0(new_n128));
  nona32aa1n03x5               g033(.a(new_n127), .b(new_n100), .c(new_n99), .d(new_n97), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(\s[10] ));
  nor042aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1d04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n03x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n03x5               g039(.a(new_n131), .b(new_n129), .c(new_n98), .d(new_n132), .o1(new_n135));
  xnrb03aa1n03x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norb02aa1n02x7               g041(.a(new_n98), .b(new_n97), .out0(new_n137));
  nor002aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nano23aa1n06x5               g044(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n140));
  and003aa1n02x5               g045(.a(new_n140), .b(new_n126), .c(new_n137), .o(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n111), .c(new_n122), .d(new_n125), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n143));
  nanb02aa1n02x5               g048(.a(new_n138), .b(new_n139), .out0(new_n144));
  oai022aa1n02x7               g049(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n145));
  nano23aa1n06x5               g050(.a(new_n144), .b(new_n133), .c(new_n145), .d(new_n98), .out0(new_n146));
  nor043aa1n03x5               g051(.a(new_n146), .b(new_n143), .c(new_n138), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nanp02aa1n24x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n142), .c(new_n147), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[13] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(\b[12] ), .o1(new_n153));
  nanp02aa1n06x5               g058(.a(new_n142), .b(new_n147), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(new_n152), .b(new_n153), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nand02aa1d12x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanb02aa1d36x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nor002aa1d32x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n24x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1d15x5               g067(.a(new_n148), .b(new_n161), .c(new_n162), .d(new_n149), .out0(new_n163));
  aoai13aa1n12x5               g068(.a(new_n162), .b(new_n161), .c(new_n152), .d(new_n153), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n160), .b(new_n165), .c(new_n154), .d(new_n163), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n160), .b(new_n165), .c(new_n154), .d(new_n163), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  nor042aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand02aa1n06x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1d36x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  oai112aa1n03x5               g076(.a(new_n166), .b(new_n171), .c(\b[14] ), .d(\a[15] ), .o1(new_n172));
  oaoi13aa1n06x5               g077(.a(new_n171), .b(new_n166), .c(\a[15] ), .d(\b[14] ), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n172), .b(new_n173), .out0(\s[16] ));
  inv020aa1n02x5               g079(.a(new_n145), .o1(new_n175));
  nona22aa1n02x5               g080(.a(new_n140), .b(new_n175), .c(new_n99), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n143), .c(new_n138), .out0(new_n177));
  nona22aa1n09x5               g082(.a(new_n163), .b(new_n171), .c(new_n159), .out0(new_n178));
  inv000aa1n02x5               g083(.a(new_n178), .o1(new_n179));
  tech160nm_fiaoi012aa1n03p5x5 g084(.a(new_n169), .b(new_n157), .c(new_n170), .o1(new_n180));
  oai013aa1n06x5               g085(.a(new_n180), .b(new_n164), .c(new_n159), .d(new_n171), .o1(new_n181));
  aoi012aa1n12x5               g086(.a(new_n181), .b(new_n177), .c(new_n179), .o1(new_n182));
  nano32aa1n03x7               g087(.a(new_n178), .b(new_n140), .c(new_n126), .d(new_n137), .out0(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n111), .c(new_n122), .d(new_n125), .o1(new_n184));
  nanp02aa1n09x5               g089(.a(new_n182), .b(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  inv040aa1d32x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d06x4               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n189), .b(new_n188), .o1(new_n193));
  oaoi03aa1n12x5               g098(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n194));
  nor002aa1n08x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand22aa1n04x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n192), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n185), .d(new_n192), .o1(new_n199));
  norb02aa1n03x4               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand22aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n02x5               g109(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n204), .o1(new_n206));
  oaoi13aa1n02x7               g111(.a(new_n206), .b(new_n198), .c(\a[19] ), .d(\b[18] ), .o1(new_n207));
  norb02aa1n03x4               g112(.a(new_n205), .b(new_n207), .out0(\s[20] ));
  nano23aa1n06x5               g113(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n192), .b(new_n209), .o1(new_n210));
  oai022aa1n02x7               g115(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n211));
  oaib12aa1n06x5               g116(.a(new_n211), .b(new_n187), .c(\b[17] ), .out0(new_n212));
  nona23aa1n09x5               g117(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n213));
  aoi012aa1n06x5               g118(.a(new_n202), .b(new_n195), .c(new_n203), .o1(new_n214));
  oai012aa1n18x5               g119(.a(new_n214), .b(new_n213), .c(new_n212), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n04x5               g121(.a(new_n216), .b(new_n210), .c(new_n182), .d(new_n184), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xorc02aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n219), .b(new_n221), .c(new_n217), .d(new_n220), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n223));
  norb02aa1n02x7               g128(.a(new_n223), .b(new_n222), .out0(\s[22] ));
  inv000aa1d42x5               g129(.a(\a[21] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  xroi22aa1d06x4               g131(.a(new_n225), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  nanp03aa1n02x5               g132(.a(new_n227), .b(new_n192), .c(new_n209), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[21] ), .o1(new_n229));
  oaoi03aa1n12x5               g134(.a(new_n226), .b(new_n229), .c(new_n219), .o1(new_n230));
  inv040aa1d30x5               g135(.a(new_n230), .o1(new_n231));
  tech160nm_fiaoi012aa1n03p5x5 g136(.a(new_n231), .b(new_n215), .c(new_n227), .o1(new_n232));
  aoai13aa1n04x5               g137(.a(new_n232), .b(new_n228), .c(new_n182), .d(new_n184), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n03x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x7               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  and002aa1n02x5               g145(.a(new_n237), .b(new_n236), .o(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x5               g147(.a(new_n242), .b(new_n227), .c(new_n192), .d(new_n209), .out0(new_n243));
  inv040aa1n02x5               g148(.a(new_n214), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n227), .b(new_n244), .c(new_n209), .d(new_n194), .o1(new_n245));
  orn002aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .o(new_n246));
  oao003aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n242), .c(new_n245), .d(new_n230), .o1(new_n248));
  tech160nm_fixorc02aa1n05x5   g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n185), .d(new_n243), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n185), .d(new_n243), .o1(new_n251));
  norb02aa1n03x4               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  tech160nm_fixorc02aa1n05x5   g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n03x5               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  inv000aa1n02x5               g160(.a(new_n253), .o1(new_n256));
  aobi12aa1n06x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  inv000aa1n02x5               g163(.a(new_n111), .o1(new_n259));
  aob012aa1n03x5               g164(.a(new_n259), .b(new_n122), .c(new_n125), .out0(new_n260));
  oabi12aa1n03x5               g165(.a(new_n181), .b(new_n147), .c(new_n178), .out0(new_n261));
  and002aa1n06x5               g166(.a(new_n254), .b(new_n249), .o(new_n262));
  nano22aa1n03x7               g167(.a(new_n228), .b(new_n241), .c(new_n262), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n261), .c(new_n260), .d(new_n183), .o1(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n265));
  aobi12aa1n06x5               g170(.a(new_n265), .b(new_n248), .c(new_n262), .out0(new_n266));
  xorc02aa1n12x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  xnbna2aa1n03x5               g172(.a(new_n267), .b(new_n266), .c(new_n264), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  inv040aa1n03x5               g174(.a(new_n269), .o1(new_n270));
  aobi12aa1n02x7               g175(.a(new_n267), .b(new_n266), .c(new_n264), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  nano22aa1n03x5               g177(.a(new_n271), .b(new_n270), .c(new_n272), .out0(new_n273));
  aobi12aa1n06x5               g178(.a(new_n263), .b(new_n182), .c(new_n184), .out0(new_n274));
  aoai13aa1n04x5               g179(.a(new_n241), .b(new_n231), .c(new_n215), .d(new_n227), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n262), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n265), .b(new_n276), .c(new_n275), .d(new_n247), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n267), .b(new_n277), .c(new_n274), .o1(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n272), .b(new_n278), .c(new_n270), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n279), .b(new_n273), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n267), .b(new_n272), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n274), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n270), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  aobi12aa1n02x7               g190(.a(new_n281), .b(new_n266), .c(new_n264), .out0(new_n286));
  nano22aa1n02x4               g191(.a(new_n286), .b(new_n283), .c(new_n284), .out0(new_n287));
  norp02aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n267), .b(new_n284), .c(new_n272), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n277), .c(new_n274), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n290), .b(new_n266), .c(new_n264), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n02x7               g203(.a(new_n298), .b(new_n266), .c(new_n264), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n298), .b(new_n277), .c(new_n274), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  xnrb03aa1n02x5               g210(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g214(.a(\b[4] ), .b(\a[5] ), .o1(new_n310));
  aoib12aa1n06x5               g215(.a(new_n310), .b(new_n122), .c(new_n124), .out0(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(new_n106), .out0(\s[6] ));
  and002aa1n02x5               g217(.a(\b[5] ), .b(\a[6] ), .o(new_n313));
  nanb02aa1n06x5               g218(.a(new_n123), .b(new_n311), .out0(new_n314));
  nona23aa1n03x5               g219(.a(new_n314), .b(new_n104), .c(new_n103), .d(new_n313), .out0(new_n315));
  aboi22aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n109), .d(new_n104), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n315), .b(new_n316), .out0(\s[7] ));
  norb02aa1n02x5               g222(.a(new_n101), .b(new_n102), .out0(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n109), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n260), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


