// Benchmark "adder" written by ABC on Wed Jul 17 14:12:47 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n291, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n311, new_n313, new_n314, new_n315, new_n316, new_n319,
    new_n320, new_n321, new_n323, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\a[10] ), .b(new_n97), .out0(new_n98));
  tech160nm_finand02aa1n05x5   g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  norp02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi022aa1d24x5               g007(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  tech160nm_finand02aa1n05x5   g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n06x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  oai022aa1d18x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  oaoi13aa1n12x5               g012(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n102), .o1(new_n108));
  tech160nm_fixorc02aa1n02p5x5 g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  nor042aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nano22aa1n03x7               g017(.a(new_n110), .b(new_n111), .c(new_n112), .out0(new_n113));
  nor042aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  and002aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .o(new_n115));
  nand42aa1n06x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n116), .b(\b[5] ), .c(\a[6] ), .o1(new_n117));
  norp03aa1n02x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .o1(new_n118));
  nand23aa1n02x5               g023(.a(new_n118), .b(new_n109), .c(new_n113), .o1(new_n119));
  nor042aa1n02x5               g024(.a(new_n115), .b(new_n114), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aob012aa1n02x5               g026(.a(new_n110), .b(\b[7] ), .c(\a[8] ), .out0(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(\b[7] ), .c(\a[8] ), .o1(new_n123));
  aoi013aa1n02x5               g028(.a(new_n123), .b(new_n113), .c(new_n120), .d(new_n121), .o1(new_n124));
  oai012aa1n18x5               g029(.a(new_n124), .b(new_n119), .c(new_n108), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n100), .b(new_n125), .c(new_n101), .o1(new_n126));
  tech160nm_finor002aa1n03p5x5 g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb03aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n100), .out0(new_n128));
  aob012aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n101), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .d(new_n99), .o1(\s[10] ));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor042aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n99), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(new_n132), .o1(new_n135));
  nanp03aa1n02x5               g040(.a(new_n129), .b(new_n99), .c(new_n133), .o1(new_n136));
  xorc02aa1n02x5               g041(.a(\a[12] ), .b(\b[11] ), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n136), .c(new_n135), .out0(\s[12] ));
  tech160nm_fixnrc02aa1n04x5   g043(.a(\b[11] ), .b(\a[12] ), .out0(new_n139));
  nanb03aa1n09x5               g044(.a(new_n132), .b(new_n99), .c(new_n131), .out0(new_n140));
  norb03aa1n02x5               g045(.a(new_n101), .b(new_n127), .c(new_n100), .out0(new_n141));
  norb03aa1n02x5               g046(.a(new_n141), .b(new_n139), .c(new_n140), .out0(new_n142));
  nanp02aa1n02x5               g047(.a(new_n125), .b(new_n142), .o1(new_n143));
  oao003aa1n02x5               g048(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .carry(new_n144));
  oai013aa1d12x5               g049(.a(new_n144), .b(new_n128), .c(new_n140), .d(new_n139), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n03x5               g051(.a(new_n143), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  tech160nm_fiaoi012aa1n05x5   g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand22aa1n04x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n145), .c(new_n125), .d(new_n142), .o1(new_n156));
  aoi012aa1d18x5               g061(.a(new_n153), .b(new_n149), .c(new_n154), .o1(new_n157));
  nor042aa1n06x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nand22aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n156), .c(new_n157), .out0(\s[15] ));
  inv000aa1d42x5               g066(.a(new_n158), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n160), .o1(new_n163));
  aoai13aa1n03x5               g068(.a(new_n162), .b(new_n163), .c(new_n156), .d(new_n157), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  inv000aa1d42x5               g070(.a(\a[17] ), .o1(new_n166));
  nano22aa1n02x4               g071(.a(new_n132), .b(new_n99), .c(new_n131), .out0(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nano23aa1n06x5               g074(.a(new_n158), .b(new_n168), .c(new_n169), .d(new_n159), .out0(new_n170));
  nand22aa1n06x5               g075(.a(new_n170), .b(new_n155), .o1(new_n171));
  nano32aa1n06x5               g076(.a(new_n171), .b(new_n141), .c(new_n167), .d(new_n137), .out0(new_n172));
  oai112aa1n02x5               g077(.a(new_n98), .b(new_n99), .c(\b[8] ), .d(\a[9] ), .o1(new_n173));
  nanp03aa1n02x5               g078(.a(new_n137), .b(new_n167), .c(new_n173), .o1(new_n174));
  inv000aa1n02x5               g079(.a(new_n157), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .c(new_n162), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n176), .b(new_n170), .c(new_n175), .o1(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n171), .c(new_n174), .d(new_n144), .o1(new_n178));
  aoi012aa1d18x5               g083(.a(new_n178), .b(new_n125), .c(new_n172), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(new_n166), .out0(\s[17] ));
  oaoi03aa1n02x5               g085(.a(\a[17] ), .b(\b[16] ), .c(new_n179), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n06x5               g087(.a(new_n125), .b(new_n172), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n171), .o1(new_n184));
  aobi12aa1n03x5               g089(.a(new_n177), .b(new_n145), .c(new_n184), .out0(new_n185));
  nand02aa1d06x5               g090(.a(new_n183), .b(new_n185), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  xroi22aa1d04x5               g092(.a(new_n166), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nona22aa1n02x4               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(new_n190));
  oaib12aa1n09x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(new_n191));
  aoi012aa1n12x5               g096(.a(new_n191), .b(new_n186), .c(new_n188), .o1(new_n192));
  nor042aa1d18x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  nanp02aa1n04x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n192), .b(new_n195), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanb02aa1n02x5               g102(.a(new_n193), .b(new_n195), .out0(new_n198));
  nor042aa1n04x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand02aa1d04x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  oaoi13aa1n02x7               g106(.a(new_n201), .b(new_n194), .c(new_n192), .d(new_n198), .o1(new_n202));
  oai112aa1n03x5               g107(.a(new_n201), .b(new_n194), .c(new_n192), .d(new_n198), .o1(new_n203));
  norb02aa1n02x7               g108(.a(new_n203), .b(new_n202), .out0(\s[20] ));
  nona23aa1n09x5               g109(.a(new_n200), .b(new_n195), .c(new_n193), .d(new_n199), .out0(new_n205));
  norb02aa1n03x5               g110(.a(new_n188), .b(new_n205), .out0(new_n206));
  aoai13aa1n12x5               g111(.a(new_n206), .b(new_n178), .c(new_n125), .d(new_n172), .o1(new_n207));
  aoi112aa1n02x7               g112(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n208));
  aoib12aa1n06x5               g113(.a(new_n208), .b(new_n187), .c(\b[17] ), .out0(new_n209));
  aoi012aa1n12x5               g114(.a(new_n199), .b(new_n193), .c(new_n200), .o1(new_n210));
  oai012aa1d24x5               g115(.a(new_n210), .b(new_n205), .c(new_n209), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  xnrc02aa1n12x5               g117(.a(\b[20] ), .b(\a[21] ), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n207), .c(new_n212), .out0(\s[21] ));
  nor042aa1n06x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n216), .b(new_n213), .c(new_n207), .d(new_n212), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[21] ), .c(\a[22] ), .out0(\s[22] ));
  tech160nm_fixnrc02aa1n04x5   g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  nor042aa1n06x5               g124(.a(new_n219), .b(new_n213), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  oao003aa1n12x5               g126(.a(\a[22] ), .b(\b[21] ), .c(new_n216), .carry(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoi012aa1n12x5               g128(.a(new_n223), .b(new_n211), .c(new_n220), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[22] ), .b(\a[23] ), .out0(new_n225));
  oaoi13aa1n09x5               g130(.a(new_n225), .b(new_n224), .c(new_n207), .d(new_n221), .o1(new_n226));
  nano22aa1n03x7               g131(.a(new_n179), .b(new_n206), .c(new_n220), .out0(new_n227));
  nano22aa1n02x4               g132(.a(new_n227), .b(new_n224), .c(new_n225), .out0(new_n228));
  norp02aa1n02x5               g133(.a(new_n226), .b(new_n228), .o1(\s[23] ));
  nor042aa1n03x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n224), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n225), .o1(new_n233));
  oai012aa1n04x7               g138(.a(new_n233), .b(new_n227), .c(new_n232), .o1(new_n234));
  tech160nm_fixnrc02aa1n05x5   g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n234), .c(new_n231), .o1(new_n236));
  nano22aa1n03x7               g141(.a(new_n226), .b(new_n231), .c(new_n235), .out0(new_n237));
  nor002aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(\s[24] ));
  nano23aa1n03x7               g143(.a(new_n193), .b(new_n199), .c(new_n200), .d(new_n195), .out0(new_n239));
  inv020aa1n02x5               g144(.a(new_n210), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n220), .b(new_n240), .c(new_n239), .d(new_n191), .o1(new_n241));
  nor042aa1n02x5               g146(.a(new_n235), .b(new_n225), .o1(new_n242));
  inv030aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  oao003aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n231), .carry(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n243), .c(new_n241), .d(new_n222), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n242), .b(new_n220), .o1(new_n247));
  tech160nm_fixnrc02aa1n04x5   g152(.a(\b[24] ), .b(\a[25] ), .out0(new_n248));
  oaoi13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n207), .d(new_n247), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n242), .b(new_n223), .c(new_n211), .d(new_n220), .o1(new_n250));
  nano32aa1n03x7               g155(.a(new_n179), .b(new_n242), .c(new_n206), .d(new_n220), .out0(new_n251));
  nano32aa1n02x4               g156(.a(new_n251), .b(new_n248), .c(new_n250), .d(new_n244), .out0(new_n252));
  norp02aa1n02x5               g157(.a(new_n249), .b(new_n252), .o1(\s[25] ));
  nor042aa1n03x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  oabi12aa1n03x5               g160(.a(new_n248), .b(new_n251), .c(new_n245), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[25] ), .b(\a[26] ), .out0(new_n257));
  aoi012aa1n03x5               g162(.a(new_n257), .b(new_n256), .c(new_n255), .o1(new_n258));
  nano22aa1n03x7               g163(.a(new_n249), .b(new_n255), .c(new_n257), .out0(new_n259));
  norp02aa1n03x5               g164(.a(new_n258), .b(new_n259), .o1(\s[26] ));
  nor002aa1n02x5               g165(.a(new_n257), .b(new_n248), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n262));
  aobi12aa1n12x5               g167(.a(new_n262), .b(new_n245), .c(new_n261), .out0(new_n263));
  inv000aa1n02x5               g168(.a(new_n206), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n261), .o1(new_n265));
  nona32aa1n09x5               g170(.a(new_n186), .b(new_n265), .c(new_n247), .d(new_n264), .out0(new_n266));
  nor042aa1n03x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  and002aa1n18x5               g172(.a(\b[26] ), .b(\a[27] ), .o(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n267), .o1(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n263), .out0(\s[27] ));
  inv000aa1d42x5               g175(.a(new_n267), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n262), .b(new_n265), .c(new_n250), .d(new_n244), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n265), .b(new_n220), .c(new_n242), .out0(new_n273));
  nano22aa1n12x5               g178(.a(new_n179), .b(new_n206), .c(new_n273), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n268), .o1(new_n275));
  tech160nm_fioai012aa1n03p5x5 g180(.a(new_n275), .b(new_n272), .c(new_n274), .o1(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoi012aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n271), .o1(new_n278));
  aoi012aa1n03x5               g183(.a(new_n268), .b(new_n266), .c(new_n263), .o1(new_n279));
  nano22aa1n03x5               g184(.a(new_n279), .b(new_n271), .c(new_n277), .out0(new_n280));
  nor002aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[28] ));
  nano22aa1n03x7               g186(.a(new_n277), .b(new_n275), .c(new_n271), .out0(new_n282));
  tech160nm_fioai012aa1n05x5   g187(.a(new_n282), .b(new_n272), .c(new_n274), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n03x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n282), .o1(new_n287));
  tech160nm_fiaoi012aa1n05x5   g192(.a(new_n287), .b(new_n266), .c(new_n263), .o1(new_n288));
  nano22aa1n03x7               g193(.a(new_n288), .b(new_n284), .c(new_n285), .out0(new_n289));
  nor002aa1n02x5               g194(.a(new_n286), .b(new_n289), .o1(\s[29] ));
  nanp02aa1n02x5               g195(.a(\b[0] ), .b(\a[1] ), .o1(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n269), .b(new_n285), .c(new_n277), .out0(new_n293));
  tech160nm_fioai012aa1n05x5   g198(.a(new_n293), .b(new_n272), .c(new_n274), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  tech160nm_fiaoi012aa1n05x5   g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n293), .o1(new_n298));
  tech160nm_fiaoi012aa1n05x5   g203(.a(new_n298), .b(new_n266), .c(new_n263), .o1(new_n299));
  nano22aa1n03x7               g204(.a(new_n299), .b(new_n295), .c(new_n296), .out0(new_n300));
  nor042aa1n03x5               g205(.a(new_n297), .b(new_n300), .o1(\s[30] ));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  norb03aa1n02x5               g207(.a(new_n282), .b(new_n296), .c(new_n285), .out0(new_n303));
  tech160nm_fioai012aa1n05x5   g208(.a(new_n303), .b(new_n272), .c(new_n274), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n305));
  aoi012aa1n03x5               g210(.a(new_n302), .b(new_n304), .c(new_n305), .o1(new_n306));
  inv000aa1n02x5               g211(.a(new_n303), .o1(new_n307));
  aoi012aa1n06x5               g212(.a(new_n307), .b(new_n266), .c(new_n263), .o1(new_n308));
  nano22aa1n03x7               g213(.a(new_n308), .b(new_n302), .c(new_n305), .out0(new_n309));
  nor002aa1n02x5               g214(.a(new_n306), .b(new_n309), .o1(\s[31] ));
  norp02aa1n02x5               g215(.a(new_n103), .b(new_n102), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(new_n311), .b(new_n106), .out0(\s[3] ));
  orn002aa1n02x5               g217(.a(\a[4] ), .b(\b[3] ), .o(new_n313));
  inv000aa1d42x5               g218(.a(new_n108), .o1(new_n314));
  tech160nm_fiao0012aa1n02p5x5 g219(.a(new_n104), .b(new_n313), .c(new_n116), .o(new_n315));
  aoib12aa1n02x5               g220(.a(new_n315), .b(new_n106), .c(new_n311), .out0(new_n316));
  aoi013aa1n02x4               g221(.a(new_n316), .b(new_n314), .c(new_n116), .d(new_n313), .o1(\s[4] ));
  xobna2aa1n03x5               g222(.a(new_n109), .b(new_n314), .c(new_n116), .out0(\s[5] ));
  xorc02aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n109), .b(new_n108), .c(\a[4] ), .d(\b[3] ), .o1(new_n320));
  aob012aa1n02x5               g225(.a(new_n320), .b(\b[4] ), .c(\a[5] ), .out0(new_n321));
  xnrc02aa1n02x5               g226(.a(new_n321), .b(new_n319), .out0(\s[6] ));
  nanb02aa1n02x5               g227(.a(new_n110), .b(new_n111), .out0(new_n323));
  nanp02aa1n02x5               g228(.a(new_n321), .b(new_n319), .o1(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n323), .b(new_n324), .c(new_n112), .out0(\s[7] ));
  aoi012aa1n02x5               g230(.a(new_n110), .b(new_n324), .c(new_n113), .o1(new_n326));
  xnrc02aa1n02x5               g231(.a(new_n326), .b(new_n120), .out0(\s[8] ));
  xorb03aa1n02x5               g232(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

